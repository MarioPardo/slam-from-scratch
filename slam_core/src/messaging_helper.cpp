#include "messaging_helper.h"
#include <sstream>
#include <limits>
#include <cmath>

// --- JSON extraction helpers ---

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

// --- Message parsers ---

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

// --- Outgoing visualization message ---

std::string createVisualizationMessage(
    const std::vector<slam::Pose2D>& odom_trajectory,
    const std::vector<slam::Pose2D>& icp_trajectory,
    const std::vector<slam::Point2D>& lidar_points,
    const std::vector<slam::LineSegment>& extracted_lines,
    const std::vector<slam::Node>& graph_nodes,
    const std::vector<slam::Edge>& graph_edges,
    const slam::Pose2D* gt_pose,
    bool clear_map,
    const std::vector<slam::Point2D>& full_map_points,
    bool map_update) {

    std::ostringstream viz_data;

    // Send only the latest pose per frame — the viewer accumulates history.
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
    viz_data << "]},\"clear_map\":" << (clear_map ? "true" : "false")
             << ",\"map_update\":" << (map_update ? "true" : "false");

    if (clear_map) {
        viz_data << ",\"full_map_points\":[";
        for (size_t i = 0; i < full_map_points.size(); i++) {
            if (i > 0) viz_data << ",";
            viz_data << "{\"x\":" << full_map_points[i].x
                     << ",\"y\":" << full_map_points[i].y << "}";
        }
        viz_data << "]";
    }

    viz_data << "}";

    return viz_data.str();
}
