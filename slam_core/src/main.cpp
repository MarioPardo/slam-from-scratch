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

// Global flag for clean shutdown
volatile sig_atomic_t running = 1;

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
    const std::vector<slam::Edge>& graph_edges) {
    
    std::ostringstream viz_data;
    viz_data << "{\"odom_trajectory\": [";
    for (size_t i = 0; i < odom_trajectory.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << odom_trajectory[i].x << ",\"y\":" << odom_trajectory[i].y 
                 << ",\"theta\":" << odom_trajectory[i].theta << "}";  // Use raw theta (flipped from before)
    }
    viz_data << "],\"icp_trajectory\":[";
    for (size_t i = 0; i < icp_trajectory.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"x\":" << icp_trajectory[i].x << ",\"y\":" << icp_trajectory[i].y 
                 << ",\"theta\":" << (-icp_trajectory[i].theta - M_PI/2.0) << "}";  // Negate and rotate 90deg CW
    }
    viz_data << "],\"lidar_points\":[";
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
                 << ",\"y\":"  << graph_nodes[i].pose.y << "}";
    }
    viz_data << "],\"edges\":[";
    for (size_t i = 0; i < graph_edges.size(); i++) {
        if (i > 0) viz_data << ",";
        viz_data << "{\"from\":" << graph_edges[i].from_id
                 << ",\"to\":"   << graph_edges[i].to_id
                 << ",\"type\":" << (graph_edges[i].edgeType == slam::LOOP_CLOSURE ? 1 : 0) << "}";
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

        // Raw occupancy grid (non-optimized), 0.1 m resolution, static 10x10 m map
        // World coverage: x in [-5, 5], y in [-5, 5] so world (0,0) is at grid center
        slam::OccupancyGrid raw_grid(0.1, 100, 100, -5.0, -5.0);
        // Optimized occupancy grid rebuilt from pose-graph keyframes
        slam::OccupancyGrid optimized_grid(0.1, 100, 100, -5.0, -5.0);
        
        // Odom trajectory
        std::vector<slam::Pose2D> odom_trajectory; //keeping for now during development
        slam::Pose2D prev_odompose_keyframe = {0.0, 0.0, 0.0};
        
        // ICP trajectory.
        std::vector<slam::Pose2D> icp_trajectory;
        slam::Pose2D keyframe_icp_pose        = {0.0, 0.0, 0.0};  // frozen at last keyframe
        std::vector<Eigen::Vector2d> prev_pointcloud_keyframe;
        
        bool first_scan = true;
        
        subscriber.subscribe("robot_state");
        std::cout << "[Main] Waiting for messages... (Press Ctrl+C to exit)" << std::endl;
        std::cout << "[Main] Mode: Odom + Lidar viz | ICP = SHADOW (passive logging)" << std::endl;
        
        int message_count = 0;
        
        // Message callback: odom map + ICP trajectory side-by-side
        auto messageCallback = [&publisher, &odometry, &odom_trajectory, &icp_trajectory,
                                 &message_count, &prev_pointcloud_keyframe, &prev_odompose_keyframe,
                                 &keyframe_icp_pose, &first_scan, &pose_graph, &raw_grid, &optimized_grid]
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

                if (!scan.ranges.empty()) {
                    world_lidar_points   = slam::transformToWorld(scan, curr_odom_pose);
                    curr_point_cloud     = slam::scanToPointCloud(scan);  // robot frame, for ICP
                }

                // ICP gating: skip ICP during turns, use odometry instead
                const double GYRO_TURN_THRESHOLD = 0.2; // rad/s
                const bool is_turning = std::abs(odom.gyro_z) > GYRO_TURN_THRESHOLD;

                slam::Pose2D curr_icp_pose = curr_odom_pose; // fallback

                if (!first_scan && !prev_pointcloud_keyframe.empty() && !curr_point_cloud.empty())
                {
                    if (is_turning) //if turning, ICP struggles so we use odom to compute pose delta
                    {
                        // Integrate frame-to-frame odom delta onto previous ICP pose
                        slam::Transform2D odom_delta = computePoseDelta(odom_trajectory.back(), curr_odom_pose);
                        curr_icp_pose = icp_trajectory.back().transform(odom_delta);
                        //curr_icp_pose.theta = odom.compass_heading;
                    }
                    else
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

                        curr_icp_pose = keyframe_icp_pose.transform(fixed_transform);
                    }
                }

                //Update states
                if (!curr_point_cloud.empty())
                {
                    first_scan = false;
                }

                // Update raw (non-optimized) occupancy grid using current ICP pose
                if (!scan.ranges.empty())
                {
                    raw_grid.updateWithScan(scan, curr_icp_pose);
                }

                //Record both trajectories
                odom_trajectory.push_back(curr_odom_pose);
                icp_trajectory.push_back(curr_icp_pose);

                // Pose graph keyframing
                if (!scan.ranges.empty())
                    if (pose_graph.tryAddKeyframe(curr_icp_pose, scan, odom.timestamp))
                    {
                        prev_pointcloud_keyframe = curr_point_cloud;
                        prev_odompose_keyframe   = curr_odom_pose;
                        keyframe_icp_pose         = curr_icp_pose;
                    }

                // Rebuild optimized occupancy grid from current pose-graph nodes
                std::vector<slam::Node> graph_nodes = pose_graph.getNodes();
                optimized_grid = slam::OccupancyGrid(0.1, 100, 100, -5.0, -5.0);
                for (const slam::Node& node : graph_nodes)
                {
                    if (!node.lidar_scan.ranges.empty())
                    {
                        optimized_grid.updateWithScan(node.lidar_scan, node.pose);
                    }
                }

                std::vector<slam::Edge> graph_edges = pose_graph.getEdges();

                // Publish: odom map + both trajectories + pose graph
                std::string viz_message = createVisualizationMessage(
                    odom_trajectory, icp_trajectory, world_lidar_points, {},
                    graph_nodes, graph_edges);
                publisher.publishMessage("visualization", viz_message);

                // Publish occupancy grid probabilities on a separate topic
                {
                    int grid_w = raw_grid.getWidth();
                    int grid_h = raw_grid.getHeight();
                    double res = raw_grid.getResolution();

                    std::ostringstream grid_stream;
                    grid_stream << "{\"width\":" << grid_w
                                << ",\"height\":" << grid_h
                                << ",\"resolution\":" << res
                                << ",\"origin_x\":-5.0"
                                << ",\"origin_y\":-5.0"
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
