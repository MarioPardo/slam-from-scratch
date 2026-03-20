#ifndef SLAM_MESSAGING_HELPER_H
#define SLAM_MESSAGING_HELPER_H

#include "types.h"
#include "feature_extractor.h"
#include <string>
#include <vector>

// JSON extraction helpers
double extractDouble(const std::string& json, const std::string& key);
int extractInt(const std::string& json, const std::string& key);
std::vector<double> extractDoubleArray(const std::string& json, const std::string& key);

// Ground truth data from GPS
struct GroundTruth {
    double x = 0, y = 0, heading = 0;
    bool valid = false;
};

// Message parsers
slam::OdometryData parseOdometryData(const std::string& message);
slam::LidarScan parseLidarScan(const std::string& message, double timestamp);
GroundTruth parseGroundTruth(const std::string& message);

// Outgoing visualization message
std::string createVisualizationMessage(
    const std::vector<slam::Pose2D>& odom_trajectory,
    const std::vector<slam::Pose2D>& icp_trajectory,
    const std::vector<slam::Point2D>& lidar_points,
    const std::vector<slam::LineSegment>& extracted_lines,
    const std::vector<slam::Node>& graph_nodes,
    const std::vector<slam::Edge>& graph_edges,
    const slam::Pose2D* gt_pose = nullptr);

#endif // SLAM_MESSAGING_HELPER_H
