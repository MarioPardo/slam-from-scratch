#ifndef SLAM_LIDAR_PROCESSOR_H
#define SLAM_LIDAR_PROCESSOR_H

#include "types.h"
#include <vector>
#include <Eigen/Dense>

namespace slam {

std::vector<Point2D> transformToWorld(const LidarScan& scan, const Pose2D& robot_pose);
std::vector<Eigen::Vector2d> scanToPointCloud(const LidarScan& scan);

} // namespace slam

#endif // SLAM_LIDAR_PROCESSOR_H
