#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include <string>

// All tunable parameters for a SLAM run, previously scattered as magic numbers
//default values used for our webots SIM. Will be overriden when testing datasets used
struct SLAMConfig {
    // ---- Robot kinematics ----
    double wheel_radius = 0.033;   // metres (TurtleBot3: 33 mm)
    double wheelbase    = 0.16;    // metres (TurtleBot3: 160 mm)

    // ---- Occupancy grid ----
    double grid_resolution = 0.01; // metres per cell
    int    grid_width      = 500;  // cells
    int    grid_height     = 500;  
    double grid_origin_x   = -2.5; // world X of cell (0,0), metres
    double grid_origin_y   = -2.5; 

    // ---- LiDAR sensor ----
    int    lidar_num_beams  = 2048;             
    double lidar_angle_min  = -M_PI;           
    double lidar_angle_max  =  M_PI;            
    double lidar_range_min  = 0.01;            
    double lidar_range_max  = 3;           

    // ---- Pose graph / keyframing ----
    double pg_min_dist_keyframe       = 0.15;  // metres — min travel before new keyframe
    double pg_min_angle_keyframe      = 0.2;   // radians — min rotation before new keyframe
    double pg_max_dist_loop_closure   = 0.3;   // metres — spatial search radius for loop candidates
    int    pg_loop_min_correspondences = 35;   // min ICP correspondences to accept a loop edge
    double pg_loop_max_icp_error      = 0.12;  // metres — max ICP residual to accept a loop edge

    // ---- ICP scan matching ----
    int    icp_max_iterations          = 100;
    double icp_convergence_epsilon     = 1e-6;
    double icp_correspondence_distance = 0.2;  // metres
    double icp_turn_threshold          = 0.1;  // rad/s gyro

    // ---- Pipeline ----
    int grid_publish_every_n = 4;  // publish occupancy grid every N messages

    // ---- ZMQ endpoints ----
    std::string zmq_sub_addr = "tcp://localhost:5555"; // subscribe to sensor data
    std::string zmq_pub_addr = "tcp://*:5556";         // publish visualization
};

// Load config from a JSON file. Any missing keys keep their struct defaults,
SLAMConfig loadConfig(const std::string& path);

#endif // SLAM_CONFIG_H
