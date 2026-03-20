#ifndef SLAM_LIDAR_PROCESSOR_H
#define SLAM_LIDAR_PROCESSOR_H

#include "types.h"
#include <vector>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

namespace slam {

std::vector<Point2D> transformToWorldFrame(const LidarScan& scan, const Pose2D& robot_pose);
std::vector<Eigen::Vector2d> scanToPointCloudRobotFrame(const LidarScan& scan);
std::vector<std::vector<Point2D>> projectNodeScansToWorldFrame(const std::vector<Node>& nodes);

} // namespace slam

#endif // SLAM_LIDAR_PROCESSOR_H
