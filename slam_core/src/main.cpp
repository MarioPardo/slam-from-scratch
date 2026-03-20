#include "zmq_bridge.h"
#include "odometry.h"
#include "lidar_processor.h"
#include "occupancy_grid.h"
#include "icp_matcher.h"
#include "feature_extractor.h"
#include "types.h"
#include "PoseGraph.h"
#include <iostream>
#include <csignal>
#include <sstream>
#include <cmath>
#include <vector>
#include <limits>
#include <cstdlib>

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

// Helper function to parse JSON string////

double extractDouble(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0.0;
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) return 0.0;
    
    size_t start = pos + 1;
    size_t end = json.find_first_of(",}", start);
    
    std::string value = json.substr(start, end - start);
    return std::stod(value);
}

int extractInt(const std::string& json, const std::string& key) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return 0;
    
    pos = json.find(":", pos);
    if (pos == std::string::npos) return 0;
    
    size_t start = pos + 1;
    size_t end = json.find_first_of(",}", start);
    
    std::string value = json.substr(start, end - start);
    return std::stoi(value);
}

std::vector<double> extractDoubleArray(const std::string& json, const std::string& key) {
    std::vector<double> result;
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return result;
    
    size_t array_start = json.find("[", pos);
    size_t array_end = json.find("]", array_start);
    if (array_start == std::string::npos || array_end == std::string::npos) return result;
    
    std::string array_content = json.substr(array_start + 1, array_end - array_start - 1);
    std::istringstream iss(array_content);
    std::string token;
    
    while (std::getline(iss, token, ',')) {
        try {
            result.push_back(std::stod(token));
        } catch (...) {
            result.push_back(std::numeric_limits<double>::infinity());
        }
    }
    
    return result;
}

slam::OdometryData parseOdometryData(const std::string& message) {
    slam::OdometryData odom;
    
    size_t header_pos = message.find("\"header\"");
    if (header_pos != std::string::npos) {
        odom.timestamp = extractDouble(message, "timestamp");
    }
    
    size_t odom_pos = message.find("\"odometry\"");
    if (odom_pos != std::string::npos) {
        odom.left_encoder = extractDouble(message, "left_encoder");
        odom.right_encoder = extractDouble(message, "right_encoder");
        odom.gyro_z = extractDouble(message, "imu_gyro_z");
        odom.compass_heading = extractDouble(message, "compass_heading");
    }
    
    return odom;
}

struct GroundTruth {
    double x = 0, y = 0, heading = 0;
    bool valid = false;
};

GroundTruth parseGroundTruth(const std::string& message) {
    GroundTruth gt;
    size_t pos = message.find("\"ground_truth\"");
    if (pos == std::string::npos) return gt;
    std::string sub = message.substr(pos);
    gt.x = extractDouble(sub, "gt_x");
    gt.y = extractDouble(sub, "gt_y");
    gt.heading = extractDouble(sub, "heading");
    gt.valid = true;
    return gt;
}

slam::LidarScan parseLidarScan(const std::string& message, double timestamp) {
    slam::LidarScan scan;
    
    size_t lidar_pos = message.find("\"lidar\"");
    if (lidar_pos != std::string::npos) {
        scan.timestamp = timestamp;
        scan.count = extractInt(message, "count");
        scan.angle_min = extractDouble(message, "angle_min");
        scan.angle_max = extractDouble(message, "angle_max");
        scan.range_min = extractDouble(message, "range_min");
        scan.range_max = extractDouble(message, "range_max");
        scan.ranges = extractDoubleArray(message, "ranges");
    }
    
    return scan;
}

std::string createVisualizationMessage(
    const std::vector<slam::Pose2D>& odom_trajectory,
    const std::vector<slam::Pose2D>& icp_trajectory,
    const std::vector<slam::Point2D>& lidar_points,
    const std::vector<slam::LineSegment>& extracted_lines,
    const std::vector<slam::Node>& graph_nodes,
    const std::vector<slam::Edge>& graph_edges,
    const slam::Pose2D* gt_pose = nullptr) {

    std::ostringstream viz_data;

    // Send only the latest pose per frame — the viewer accumulates history.
    // Sending full trajectory arrays caused the message to grow indefinitely.
    viz_data << "{";
    if (!odom_trajectory.empty()) {
        const auto& op = odom_trajectory.back();
        viz_data << "\"odom_pose\":{\"x\":" << op.x << ",\"y\":" << op.y
                 << ",\"theta\":" << op.theta << "},";
    }
    if (!icp_trajectory.empty()) {
        const auto& ip = icp_trajectory.back();
        viz_data << "\"icp_pose\":{\"x\":" << ip.x << ",\"y\":" << ip.y
                 << ",\"theta\":" << (-ip.theta - M_PI/2.0) << "},";
    }
    if (gt_pose) {
        viz_data << "\"gt_pose\":{\"x\":" << gt_pose->x << ",\"y\":" << gt_pose->y
                 << ",\"theta\":" << gt_pose->theta << "},";
    }

    viz_data << "\"lidar_points\":[";
    for (size_t i = 0; i < lidar_points.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << lidar_points[i].x 
                 << ",\"y\":" << lidar_points[i].y << "}";
    }
    viz_data << "],\"extracted_lines\":[";
    for (size_t i = 0; i < extracted_lines.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"start\":{\"x\":" << extracted_lines[i].start.x() 
                 << ",\"y\":" << extracted_lines[i].start.y() 
                 << "},\"end\":{\"x\":" << extracted_lines[i].end.x()
                 << ",\"y\":" << extracted_lines[i].end.y() << "}}";
    }
    viz_data << "],\"pose_graph\":{\"nodes\":[";
    for (size_t i = 0; i < graph_nodes.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"id\":" << graph_nodes[i].id
                 << ",\"x\":"  << graph_nodes[i].pose.x
                 << ",\"y\":"  << graph_nodes[i].pose.y
                 << ",\"theta\":"<< graph_nodes[i].pose.theta << "}";
    }
    viz_data << "],\"edges\":[";
    for (size_t i = 0; i < graph_edges.size(); i++) {
        if (i > 0) viz_data << ",";
        double mdx = graph_edges[i].transform.translation(0);
        double mdy = graph_edges[i].transform.translation(1);
        double mdth = std::atan2(graph_edges[i].transform.rotation(1,0), graph_edges[i].transform.rotation(0,0));
        viz_data << "{\"from\":" << graph_edges[i].from_id
                 << ",\"to\":"   << graph_edges[i].to_id
                 << ",\"type\":" << (graph_edges[i].edgeType == slam::LOOP_CLOSURE ? 1 : 0)
                 << ",\"meas\":{\"dx\":"<<mdx<<",\"dy\":"<<mdy<<",\"dth\":"<<mdth<<"}}";
    }
    viz_data << "]}}";

    return viz_data.str();
}



///////

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

        // Ground truth tracking
        GroundTruth gt_origin;
        bool gt_origin_set = false;
        
        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        std::cout << "[Main] Mode: Odom + Lidar viz | ICP = SHADOW (passive logging)" << std::endl;
        
        int message_count = 0;
        
        // Message callback: odom map + ICP trajectory side-by-side
        auto messageCallback = [&publisher, &odometry, &odom_trajectory, &icp_trajectory,
                                 &message_count, &prev_pointcloud_keyframe, &prev_odompose_keyframe,
                                 &keyframe_icp_pose, &first_scan, &pose_graph, &raw_grid, &optimized_grid,
                                 &gt_origin, &gt_origin_set]
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

                        // Run ICP: align current scan onto previous keyframe scan
                        slam::ICPResult icp = alignPointClouds(
                            prev_pointcloud_keyframe,
                            curr_point_cloud,
                            odom_delta,         // initial guess
                            100,
                            1e-6,
                            0.2
                        );

                        // ICP returns rotation in neg angle (not sure why lol)
                        double raw_angle   = std::atan2(icp.transform.rotation(1, 0), icp.transform.rotation(0, 0));
                        double fixed_angle = -raw_angle;
                        Eigen::Matrix2d fixed_rotation;
                        fixed_rotation << std::cos(fixed_angle), -std::sin(fixed_angle),
                                          std::sin(fixed_angle),  std::cos(fixed_angle);
                        slam::Transform2D fixed_transform(fixed_rotation, icp.transform.translation);

                        // Only use ICP correction if converged, else keep propagated pose.
                        if(icp.converged)
                            curr_icp_pose = keyframe_icp_pose.transform(fixed_transform);
                    }
                }

                //Update states
                if (!curr_point_cloud.empty())
                    first_scan = false;
                
                // Update world map using icp pose
                if (!scan.ranges.empty())
                {
                    world_lidar_points = slam::transformToWorld(scan, curr_icp_pose);
                    raw_grid.updateWithScan(scan, curr_odom_pose); //I think I shold use world_cloud_points and not have to do conversion inside updateWithScan

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
                    if (pose_graph.tryAddKeyframe(curr_icp_pose, scan, odom.timestamp, odom_rel_for_graph, &optimization_happened))
                    {
                        keyframe_added = true;
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
                            optimized_grid.updateWithScan(node.lidar_scan, node.pose); //TODO for each node project scan with new pose
                        }
                    }
                }

                std::vector<slam::Edge> graph_edges = pose_graph.getEdges();

                // Publish: odom map + both trajectories + pose graph + ground truth
                std::string viz_message = createVisualizationMessage(
                    odom_trajectory, icp_trajectory, world_lidar_points, {},
                    graph_nodes, graph_edges,
                    gt_available ? &gt_slam_pose : nullptr);
                publisher.publishMessage("visualization", viz_message);
                {
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
