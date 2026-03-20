#ifndef SLAM_POSEGRAPH_H
#define SLAM_POSEGRAPH_H

#include "types.h"
#include <vector>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

namespace slam {

class PoseGraph{

    double minDistNewKeyframe  = 0.15;   // 15 cm
    double minAngleNewKeyframe = 0.2;  // ~20 deg

    //loop closure
    double maxDistLoopClosure = 0.2;
    int recentNodeExclusion = 10;
    int maxLoopClosureCandidates = 6;
    int minKeyframesBetweenLoopClosures = 3;
    int minKeyframesBetweenOptimizations = 10;
    double loopClosure_ICPMaxError = 0.05;
    int loopClosure_ICPMinCorrespondences = 60;
    
    //capping loop closure info
    double loopClosure_minSigma = 0.02; // minimum assumed sigma (m) for loop closures
    double loopClosure_maxInformation = 400.0; //sigma 1/root(400) = 5cm max certainty

    //loop closure outlier rejection


    int nodeID = 0;
    int keyframesSinceLastLoopClosure = 5;
    int keyframesSinceLastOptimization = 10;

    std::vector<slam::Node> nodes;
    std::vector<slam::Edge> edges;
    std::vector<std::vector<slam::Point2D>> optimized_projected_scans_world;

    public:
        bool tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp, const Transform2D& odom_rel_transform, bool* optimization_happened = nullptr);
        std::vector<slam::Node> getNodes() const {return nodes;}
        std::vector<slam::Edge> getEdges() const {return edges;}
        Pose2D getLastNodePose() const { return nodes.back().pose; }
        const std::vector<std::vector<slam::Point2D>>& getOptimizedProjectedScansWorld() const { return optimized_projected_scans_world; }

    private:
        bool shouldAddKeyframe(const Pose2D& pose); //compare to latest keyframe and check if we pass angle or dist threshholds
        std::vector<Edge> detectLoopClosures(const Node& queryNode);
        void refreshOptimizedProjectedScans();

    };


}



#endif // SLAM_POSEGRAPH_H
