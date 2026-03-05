#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include <vector>
#include <Eigen/Dense>
#include <Eigen/SVD>

namespace slam {

struct Transform2D {
    Eigen::Matrix2d rotation;     
    Eigen::Vector2d translation; 
    
    //constructors
    Transform2D() : rotation(Eigen::Matrix2d::Identity()), 
                    translation(Eigen::Vector2d::Zero()) {}
    
    Transform2D(const Eigen::Matrix2d& R, const Eigen::Vector2d& t) 
        : rotation(R), translation(t) {}
    
    Eigen::Vector2d apply(const Eigen::Vector2d& point) const {
        return rotation * point + translation;
    }
};

struct Pose2D {
    double x;     
    double y;     
    double theta;  
    
    Pose2D() : x(0.0), y(0.0), theta(0.0) {}
    Pose2D(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}

    Pose2D transform(const Transform2D& T) const {
        // Create rotation matrix for current orientation
        double cos_theta = std::cos(theta);
        double sin_theta = std::sin(theta);
        Eigen::Matrix2d R_current;
        R_current << cos_theta, -sin_theta,
                     sin_theta,  cos_theta;
        
        // Apply relative transform
        Eigen::Vector2d current_pos(x, y);
        Eigen::Vector2d translation_global = R_current * T.translation;
        Eigen::Vector2d new_pos = current_pos + translation_global;
        
        double delta_theta = std::atan2(T.rotation(1, 0), T.rotation(0, 0));
        double new_theta = theta + delta_theta;

        return Pose2D(new_pos(0), new_pos(1), new_theta);
    }
};

struct OdometryData {
    double timestamp;         
    double left_encoder;      
    double right_encoder;    
    double gyro_z;            // rad/s 
    double compass_heading;   // radians (absolute heading)
    
    OdometryData() : timestamp(0.0), left_encoder(0.0), right_encoder(0.0), 
                     gyro_z(0.0), compass_heading(0.0) {}
};


struct LidarScan {
    double timestamp;           
    std::vector<double> ranges; 
    int count;                  // number of rays
    double angle_min;           
    double angle_max;           
    double range_min;           
    double range_max;          
    
    LidarScan() : timestamp(0.0), count(0), angle_min(0.0), 
                  angle_max(0.0), range_min(0.0), range_max(0.0) {}
};

struct Point2D {
    double x; 
    double y; 
    
    Point2D() : x(0.0), y(0.0) {}
    Point2D(double x_, double y_) : x(x_), y(y_) {}
};

struct Node {
    int id;
    double timestamp;
    Pose2D pose;
    LidarScan lidar_scan;
};

enum EdgeType{MOVEMENT, LOOP_CLOSURE};

struct Edge{
    int from_id;
    int to_id;
    EdgeType edgeType;
    Transform2D transform;
};


}


// namespace slam
#endif // SLAM_TYPES_H
