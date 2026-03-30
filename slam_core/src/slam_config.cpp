#include "slam_config.h"
#include "messaging_helper.h"
#include <fstream>
#include <iostream>

namespace {

bool hasKey(const std::string& json, const std::string& key) {
    return json.find("\"" + key + "\"") != std::string::npos;
}

std::string extractString(const std::string& json, const std::string& key, const std::string& def) {
    size_t pos = json.find("\"" + key + "\"");
    if (pos == std::string::npos) return def;
    pos = json.find("\"", pos + key.size() + 2); // opening quote of value
    if (pos == std::string::npos) return def;
    size_t end = json.find("\"", pos + 1);
    if (end == std::string::npos) return def;
    return json.substr(pos + 1, end - pos - 1);
}

} // namespace

SLAMConfig loadConfig(const std::string& path) {
    SLAMConfig cfg; // all fields start at struct defaults

    std::ifstream file(path);
    if (!file.is_open()) {
        std::cerr << "[SLAMConfig] Cannot open \"" << path << "\" — using defaults\n";
        return cfg;
    }

    std::string json((std::istreambuf_iterator<char>(file)),
                      std::istreambuf_iterator<char>());

    // Only override a field when the key is present — missing keys keep struct defaults.
    auto maybeDouble = [&](const std::string& key, double& field) {
        if (hasKey(json, key)) field = extractDouble(json, key);
    };
    auto maybeInt = [&](const std::string& key, int& field) {
        if (hasKey(json, key)) field = extractInt(json, key);
    };

    maybeDouble("wheel_radius",                cfg.wheel_radius);
    maybeDouble("wheelbase",                   cfg.wheelbase);
    maybeDouble("grid_resolution",             cfg.grid_resolution);
    maybeInt   ("grid_width",                  cfg.grid_width);
    maybeInt   ("grid_height",                 cfg.grid_height);
    maybeDouble("grid_origin_x",               cfg.grid_origin_x);
    maybeDouble("grid_origin_y",               cfg.grid_origin_y);
    maybeInt   ("lidar_num_beams",             cfg.lidar_num_beams);
    maybeDouble("lidar_angle_min",             cfg.lidar_angle_min);
    maybeDouble("lidar_angle_max",             cfg.lidar_angle_max);
    maybeDouble("lidar_range_min",             cfg.lidar_range_min);
    maybeDouble("lidar_range_max",             cfg.lidar_range_max);
    maybeDouble("pg_min_dist_keyframe",        cfg.pg_min_dist_keyframe);
    maybeDouble("pg_min_angle_keyframe",       cfg.pg_min_angle_keyframe);
    maybeDouble("pg_max_dist_loop_closure",    cfg.pg_max_dist_loop_closure);
    maybeInt   ("pg_loop_min_correspondences", cfg.pg_loop_min_correspondences);
    maybeDouble("pg_loop_max_icp_error",       cfg.pg_loop_max_icp_error);
    maybeInt   ("icp_max_iterations",          cfg.icp_max_iterations);
    maybeDouble("icp_convergence_epsilon",     cfg.icp_convergence_epsilon);
    maybeDouble("icp_correspondence_distance", cfg.icp_correspondence_distance);
    maybeDouble("icp_turn_threshold",          cfg.icp_turn_threshold);
    maybeInt   ("grid_publish_every_n",        cfg.grid_publish_every_n);

    cfg.zmq_sub_addr = extractString(json, "zmq_sub_addr", cfg.zmq_sub_addr);
    cfg.zmq_pub_addr = extractString(json, "zmq_pub_addr", cfg.zmq_pub_addr);

    std::cout << "[SLAMConfig] Loaded from \"" << path << "\"\n";
    return cfg;
}
