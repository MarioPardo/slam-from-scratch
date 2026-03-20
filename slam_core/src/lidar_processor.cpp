#include "lidar_processor.h"
#include <cmath>
#include <limits>

namespace slam {

//Transform a single LIDAR point from robot(polar) frame to world(cartesian) frame
//TODO turn into more efficient matrix operation
Point2D transformPoint(double range, double angle, const Pose2D& pose) {
    double x_robot = range * std::cos(angle);
    double y_robot = range * std::sin(angle);
    
    double cos_theta = std::cos(pose.theta);
    double sin_theta = std::sin(pose.theta);
    
    Point2D world_point;
    world_point.x = pose.x + x_robot * cos_theta - y_robot * sin_theta;
    world_point.y = pose.y + x_robot * sin_theta + y_robot * cos_theta;
    
    return world_point;
}

//Transform LIDAR a whole lidar scan points from robot frame to world frame
std::vector<Point2D> transformToWorldFrame(const LidarScan& scan, const Pose2D& robot_pose) {
    std::vector<Point2D> world_points;
    world_points.reserve(scan.count);
    
    if (scan.ranges.empty()) {
        return world_points;
    }
    
    double angle_increment = (scan.angle_max - scan.angle_min) / (scan.count - 1);
    
    for (int i = 0; i < scan.count; i++) 
    {
        double range = scan.ranges[i];
        
        if (std::isinf(range) || std::isnan(range)) 
            continue;
        
        if (range < scan.range_min || range > scan.range_max) 
            continue;
        
        // Webots LiDAR: ranges[0] is leftmost (angle_max), ranges[last] is rightmost (angle_min)
        double angle = scan.angle_max - i * angle_increment;

        Point2D world_point = transformPoint(range, angle, robot_pose);
        world_points.push_back(world_point);
    }
    
    return world_points;
}


std::vector<Eigen::Vector2d> scanToPointCloudRobotFrame(const LidarScan& scan) {
    std::vector<Eigen::Vector2d> points;

    double angleIncrement = (scan.angle_max - scan.angle_min) / (scan.count - 1);
    
    for(size_t i = 0; i < scan.ranges.size(); ++i)
    {
        double scanRange = scan.ranges[i];
        
        if (std::isinf(scanRange) || std::isnan(scanRange)) 
            continue;
        if( scanRange < scan.range_min || scanRange > scan.range_max)
            continue;   

        // Webots LiDAR: ranges[0] is leftmost (angle_max), ranges[last] is rightmost (angle_min)
        // Reverse the angle mapping: start from angle_max and decrement
        double angle = scan.angle_max - i * angleIncrement;

        double x = scanRange * std::cos(angle);
        double y = scanRange * std::sin(angle);

        points.push_back(Eigen::Vector2d(x,y));
    }

    return points;
}

std::vector<std::vector<Point2D>> projectNodeScansToWorldFrame(const std::vector<Node>& nodes) {
    std::vector<std::vector<Point2D>> projected;
    projected.reserve(nodes.size());

    for (const Node& node : nodes) {
        if (node.lidar_scan.ranges.empty()) {
            projected.emplace_back();
            continue;
        }
        projected.push_back(transformToWorldFrame(node.lidar_scan, node.pose));
    }

    return projected;
}

} // namespace slam
