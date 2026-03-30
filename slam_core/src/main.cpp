#include "slam_config.h"
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

int main(int argc, char* argv[]) {
    std::cout << "=== SLAM Core - ZMQ Bidirectional Communication ===" << std::endl;

    SLAMConfig cfg;
    if (argc > 1) cfg = loadConfig(argv[1]);

    // Setup signal handling for clean shutdown (Ctrl+C)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    try {
        slam::ZMQSubscriber subscriber(cfg.zmq_sub_addr);
        slam::ZMQPublisher publisher(cfg.zmq_pub_addr);

        slam::OdometryProcessor odometry(cfg.wheel_radius, cfg.wheelbase);
        slam::PoseGraph pose_graph;
        pose_graph.configure(cfg);

        // Occupancy grids: odom_grid uses raw odometry poses, icp_grid uses ICP-corrected poses
        slam::OccupancyGrid odom_grid(cfg.grid_resolution, cfg.grid_width, cfg.grid_height, cfg.grid_origin_x, cfg.grid_origin_y);
        slam::OccupancyGrid icp_grid(cfg.grid_resolution, cfg.grid_width, cfg.grid_height, cfg.grid_origin_x, cfg.grid_origin_y);

        // Odom trajectory
        std::vector<slam::Pose2D> odom_trajectory;
        slam::Pose2D prev_odompose_keyframe = {0.0, 0.0, 0.0};

        // ICP trajectory.
        std::vector<slam::Pose2D> icp_trajectory;
        slam::Pose2D keyframe_icp_pose        = {0.0, 0.0, 0.0};
        std::vector<Eigen::Vector2d> prev_pointcloud_keyframe;

        bool first_scan = true;
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
                                 &keyframe_icp_pose, &first_scan, &pose_graph, &odom_grid, &icp_grid,
                                 &gt_origin, &gt_origin_set,
                                 &pending_full_map_points, &clear_map_frames_remaining, &cfg]
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
                    curr_point_cloud = slam::scanToPointCloudRobotFrame(scan);


                // ICP gating: skip ICP during turns, use odometry instead
                const double GYRO_TURN_THRESHOLD = cfg.icp_turn_threshold;
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

                        // Run ICP
                        slam::ICPResult icp = alignPointClouds(
                            curr_point_cloud,
                            prev_pointcloud_keyframe,
                            odom_delta,
                            cfg.icp_max_iterations,
                            cfg.icp_convergence_epsilon,
                            cfg.icp_correspondence_distance
                        );

                        if(icp.converged) {
                            curr_icp_pose = keyframe_icp_pose.transform(icp.transform);
                        }
                    }
                }

                //Update states
                if (!curr_point_cloud.empty() && !is_turning)
                    first_scan = false;

                // Update both grids — skip distorted scans during rotation
                if (!scan.ranges.empty() && !is_turning)
                {
                    world_lidar_points = slam::transformToWorldFrame(scan, curr_icp_pose);
                    odom_grid.updateWithScan(scan, curr_odom_pose);
                    icp_grid.updateWithScan(scan, curr_icp_pose);
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
                if (!scan.ranges.empty() && !is_turning)
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
                        slam::Pose2D pre_opt_pose = curr_icp_pose;
                        slam::Pose2D optimized_pose = pose_graph.getLastNodePose();
                        keyframe_icp_pose     = optimized_pose;
                        icp_trajectory.back() = optimized_pose;

                        if (gt_available) {
                            double pre_err = std::hypot(pre_opt_pose.x - gt_slam_pose.x,
                                                        pre_opt_pose.y - gt_slam_pose.y);
                            double post_err = std::hypot(optimized_pose.x - gt_slam_pose.x,
                                                         optimized_pose.y - gt_slam_pose.y);
                            std::cout << "[Metrics] Optimization error removed: "
                                      << (pre_err - post_err)
                                      << " m (pre=" << pre_err
                                      << " m, post=" << post_err << " m)" << std::endl;
                        }
                    }
                }

                // Rebuild ICP grid from corrected poses after loop closure optimization.
                std::vector<slam::Node> graph_nodes = pose_graph.getNodes();
                if (optimization_happened)
                {
                    icp_grid = slam::OccupancyGrid(cfg.grid_resolution, cfg.grid_width, cfg.grid_height, cfg.grid_origin_x, cfg.grid_origin_y);
                    for (const slam::Node& node : graph_nodes)
                    {
                        if (!node.lidar_scan.ranges.empty())
                        {
                            icp_grid.updateWithScan(node.lidar_scan, node.pose);
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
                if ((message_count % cfg.grid_publish_every_n) == 0 || optimization_happened) {
                    int grid_w = odom_grid.getWidth();
                    int grid_h = odom_grid.getHeight();
                    double res = odom_grid.getResolution();

                    std::ostringstream grid_stream;
                    grid_stream << "{\"width\":" << grid_w
                                << ",\"height\":" << grid_h
                                << ",\"resolution\":" << res
                                << ",\"origin_x\":" << odom_grid.getOriginX()
                                << ",\"origin_y\":" << odom_grid.getOriginY()
                                << ",\"cells\":[";

                    // Left panel: odometry-only map
                    for (int y = 0; y < grid_h; ++y)
                    {
                        if (y > 0) grid_stream << ",";
                        grid_stream << "[";
                        for (int x = 0; x < grid_w; ++x)
                        {
                            if (x > 0) grid_stream << ",";
                            grid_stream << odom_grid.getProbability(x, y);
                        }
                        grid_stream << "]";
                    }

                    grid_stream << "],\"optimized_cells\":[";

                    // Right panel: ICP-corrected map
                    for (int y = 0; y < grid_h; ++y)
                    {
                        if (y > 0) grid_stream << ",";
                        grid_stream << "[";
                        for (int x = 0; x < grid_w; ++x)
                        {
                            if (x > 0) grid_stream << ",";
                            grid_stream << icp_grid.getProbability(x, y);
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
