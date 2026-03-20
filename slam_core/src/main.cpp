#include "zmq_bridge.h"
#include "odometry.h"
#include "lidar_processor.h"
#include "occupancy_grid.h"
#include "icp_matcher.h"
#include "messaging_helper.h"
#include "types.h"
#include "PoseGraph.h"
#include "scan_capture.h"
#include <iostream>
#include <csignal>
#include <sstream>
#include <cmath>
#include <vector>

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

namespace {
double normalizeAngle(double a) {
    a = std::fmod(a + M_PI, 2 * M_PI);
    if (a < 0) a += 2 * M_PI;
    return a - M_PI;
}
}

void signalHandler(int signal) {
    std::cout << "\n[Main] Caught signal " << signal << ", shutting down..." << std::endl;
    running = 0;
}

int main(int /*argc*/, char* /*argv*/[]) {
    std::cout << "=== SLAM Core - ZMQ Bidirectional Communication ===" << std::endl;

    // Setup signal handling for clean shutdown (Ctrl+C)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        slam::ZMQSubscriber subscriber("tcp://localhost:5555");
        slam::ZMQPublisher publisher("tcp://*:5556");

        // TurtleBot parameters
        // Wheel radius: 33mm = 0.033m
        // Wheelbase: 160mm = 0.16m

        //Set up sensor processors
        slam::OdometryProcessor odometry(0.033, 0.16);
        slam::PoseGraph pose_graph;

        // ccupancy grid
        slam::OccupancyGrid raw_grid(0.025, 200, 200, -2.5, -2.5);
        slam::OccupancyGrid optimized_grid(0.025, 200, 200, -2.5, -2.5);

        // Odom trajectory
        std::vector<slam::Pose2D> odom_trajectory; //keeping for now during development
        slam::Pose2D prev_odompose_keyframe = {0.0, 0.0, 0.0};

        // ICP trajectory.
        std::vector<slam::Pose2D> icp_trajectory;
        slam::Pose2D keyframe_icp_pose        = {0.0, 0.0, 0.0};
        std::vector<Eigen::Vector2d> prev_pointcloud_keyframe;

        bool first_scan = true;
        const int grid_publish_every_n_messages = 4;
        std::vector<slam::Point2D> pending_full_map_points;
        int clear_map_frames_remaining = 0;

        // Ground truth tracking
        GroundTruth gt_origin;
        bool gt_origin_set = false;

        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;

        int message_count = 0;

        auto messageCallback = [&publisher, &odometry, &odom_trajectory, &icp_trajectory,
                                 &message_count, &prev_pointcloud_keyframe, &prev_odompose_keyframe,
                                 &keyframe_icp_pose, &first_scan, &pose_graph, &raw_grid, &optimized_grid,
                                 &gt_origin, &gt_origin_set,
                                 &pending_full_map_points, &clear_map_frames_remaining]
                                (const std::string& /*topic*/, const std::string& message)
        {
            message_count++;

            try {

                //Odometry
                slam::OdometryData odom = parseOdometryData(message);
                slam::Pose2D curr_odom_pose = odometry.update(odom);

                //Lidar: world-frame points for viz, robot-frame cloud for ICP
                slam::LidarScan scan = parseLidarScan(message, odom.timestamp);
                std::vector<slam::Point2D> world_lidar_points;
                std::vector<Eigen::Vector2d> curr_point_cloud;

                if (!scan.ranges.empty())
                    curr_point_cloud     = slam::scanToPointCloud(scan);  // robot frame, for ICP


                // ICP gating: skip ICP during turns, use odometry instead
                const double GYRO_TURN_THRESHOLD = 0.1; // rad/s
                const bool is_turning = std::abs(odom.gyro_z) > GYRO_TURN_THRESHOLD;

                slam::Pose2D curr_icp_pose = curr_odom_pose;

                if (!first_scan && !odom_trajectory.empty() && !icp_trajectory.empty())
                {
                    //Default to using odom for ICP
                    slam::Transform2D odom_ff_delta = computePoseDelta(odom_trajectory.back(), curr_odom_pose);
                    curr_icp_pose = icp_trajectory.back().transform(odom_ff_delta);

                    //if can use ICP, use it
                    if (!is_turning && !prev_pointcloud_keyframe.empty() && !curr_point_cloud.empty())
                    {
                        slam::Transform2D odom_delta = computePoseDelta(prev_odompose_keyframe, curr_odom_pose);

                        // ICP finds Transform for source_scan to target_scan, which is the INVERSE of the
                        // robot's forward motion. Give it the inverted odometry as initial guess.
                        Eigen::Matrix2d Rinv_od = odom_delta.rotation.transpose();
                        slam::Transform2D icp_initial(Rinv_od, -(Rinv_od * odom_delta.translation));

                        // Run ICP: align current scan onto previous keyframe scan
                        slam::ICPResult icp = alignPointClouds(
                            prev_pointcloud_keyframe,
                            curr_point_cloud,
                            icp_initial,
                            100,
                            1e-6,
                            0.2
                        );

                        // Invert ICP result to recover the robot's forward motion, then apply.
                        if(icp.converged) {
                            Eigen::Matrix2d Rinv = icp.transform.rotation.transpose();
                            slam::Transform2D forward(Rinv, -(Rinv * icp.transform.translation));
                            curr_icp_pose = keyframe_icp_pose.transform(forward);
                        }
                    }
                }

                //Update states
                if (!curr_point_cloud.empty())
                    first_scan = false;

                // Update world map using icp pose
                if (!scan.ranges.empty())
                {
                    world_lidar_points = slam::transformToWorld(scan, curr_icp_pose);
                    raw_grid.updateWithScan(scan, curr_odom_pose);
                }

                //Record both trajectories
                odom_trajectory.push_back(curr_odom_pose);
                icp_trajectory.push_back(curr_icp_pose);

                // Ground truth: convert GPS world coords into SLAM frame
                GroundTruth gt = parseGroundTruth(message);
                slam::Pose2D gt_slam_pose;
                bool gt_available = false;
                if (gt.valid) {
                    if (!gt_origin_set) {
                        gt_origin = gt;
                        gt_origin_set = true;
                        std::cout << "[GT] Origin: GPS(" << gt.x << ", " << gt.y
                                  << ")  heading=" << gt.heading << std::endl;
                    }
                    double dx = gt.x - gt_origin.x;
                    double dy = gt.y - gt_origin.y;
                    double h0 = gt_origin.heading;
                    gt_slam_pose.x     =  std::cos(h0)*dx + std::sin(h0)*dy;
                    gt_slam_pose.y     = -std::sin(h0)*dx + std::cos(h0)*dy;
                    gt_slam_pose.theta = normalizeAngle(gt.heading - gt_origin.heading);
                    gt_available = true;
                }

                // Normalize ICP theta to [-pi, pi]
                curr_icp_pose.theta = normalizeAngle(curr_icp_pose.theta);

                // Pose graph keyframing
                bool optimization_happened = false;
                bool keyframe_added = false;
                slam::Transform2D odom_rel_for_graph = computePoseDelta(prev_odompose_keyframe, curr_odom_pose);
                if (!scan.ranges.empty())
                    keyframe_added = pose_graph.tryAddKeyframe(curr_icp_pose, scan, odom.timestamp, odom_rel_for_graph, &optimization_happened);

                if (keyframe_added)
                {
                    // Capture scan pair before prev is overwritten (gated by SLAM_CAPTURE_DIR)
                    static int capture_idx = 0;
                    static const char* cap_dir = std::getenv("SLAM_CAPTURE_DIR");
                    if (cap_dir && !prev_pointcloud_keyframe.empty())
                        slam::saveScanPair(prev_pointcloud_keyframe, curr_point_cloud,
                            std::string(cap_dir) + "/pair_" + std::to_string(capture_idx++) + ".json");

                    prev_pointcloud_keyframe = curr_point_cloud;
                    prev_odompose_keyframe   = curr_odom_pose;
                    keyframe_icp_pose        = curr_icp_pose;

                    //Make ICP track the latest optimized pose
                    if (optimization_happened)
                    {
                        slam::Pose2D optimized_pose = pose_graph.getLastNodePose();
                        keyframe_icp_pose     = optimized_pose;
                        icp_trajectory.back() = optimized_pose;
                    }
                }

                // Rebuild optimized occupancy grid only when optimization has corrected poses.
                std::vector<slam::Node> graph_nodes = pose_graph.getNodes();
                if (optimization_happened)
                {
                    optimized_grid = slam::OccupancyGrid(0.025, 200, 200, -2.5, -2.5);
                    for (const slam::Node& node : graph_nodes)
                    {
                        if (!node.lidar_scan.ranges.empty())
                        {
                            optimized_grid.updateWithScan(node.lidar_scan, node.pose);
                        }
                    }
                }

                std::vector<slam::Edge> graph_edges = pose_graph.getEdges();

                // When optimization fires, cache the full optimized map and broadcast
                // it for several frames so CONFLATE can't silently drop the clear signal.
                if (optimization_happened) {
                    pending_full_map_points.clear();
                    for (const auto& scan_pts : pose_graph.getOptimizedProjectedScansWorld())
                        for (const auto& pt : scan_pts)
                            pending_full_map_points.push_back(pt);
                    clear_map_frames_remaining = 10;
                }

                bool send_clear = (clear_map_frames_remaining > 0);
                if (clear_map_frames_remaining > 0) --clear_map_frames_remaining;

                // Publish visualization + grid data
                std::string viz_message = createVisualizationMessage(
                    odom_trajectory, icp_trajectory, world_lidar_points, {},
                    graph_nodes, graph_edges,
                    gt_available ? &gt_slam_pose : nullptr,
                    send_clear,
                    send_clear ? pending_full_map_points : std::vector<slam::Point2D>{},
                    keyframe_added);
                publisher.publishMessage("visualization", viz_message);
                if ((message_count % grid_publish_every_n_messages) == 0 || optimization_happened) {
                    int grid_w = raw_grid.getWidth();
                    int grid_h = raw_grid.getHeight();
                    double res = raw_grid.getResolution();

                    std::ostringstream grid_stream;
                    grid_stream << "{\"width\":" << grid_w
                                << ",\"height\":" << grid_h
                                << ",\"resolution\":" << res
                                << ",\"origin_x\":" << raw_grid.getOriginX()
                                << ",\"origin_y\":" << raw_grid.getOriginY()
                                << ",\"cells\":[";

                    for (int y = 0; y < grid_h; ++y)
                    {
                        if (y > 0) grid_stream << ",";
                        grid_stream << "[";
                        for (int x = 0; x < grid_w; ++x)
                        {
                            if (x > 0) grid_stream << ",";
                            grid_stream << raw_grid.getProbability(x, y);
                        }
                        grid_stream << "]";
                    }

                    grid_stream << "],\"optimized_cells\":[";

                    for (int y = 0; y < grid_h; ++y)
                    {
                        if (y > 0) grid_stream << ",";
                        grid_stream << "[";
                        for (int x = 0; x < grid_w; ++x)
                        {
                            if (x > 0) grid_stream << ",";
                            grid_stream << optimized_grid.getProbability(x, y);
                        }
                        grid_stream << "]";
                    }

                    grid_stream << "]}";

                    publisher.publishMessage("grid_raw", grid_stream.str());
                }

            } catch (const std::exception& e) {
                std::cerr << "[Main] Error: " << e.what() << std::endl;
            }
        };

        // Listen for messages
        while (running) {
            subscriber.receiveMessage(messageCallback, 1000); // 1s timeout
        }

        std::cout << "[Main] Exiting gracefully" << std::endl;
        std::cout << "[Main] Total messages processed: " << message_count << std::endl;

     } catch (const std::exception& e) {
        std::cerr << "[Main] Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
