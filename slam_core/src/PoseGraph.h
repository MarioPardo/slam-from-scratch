#ifndef SLAM_POSEGRAPH_H
#define SLAM_POSEGRAPH_H

#include "types.h"
#include <vector>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>

namespace slam {

class PoseGraph{

    double minDistNewKeyframe  = 0.2;   // 20 cm
    double minAngleNewKeyframe = 0.06;  // ~4 deg

    //loop closure
    double maxDistLoopClosure = 0.2; //0.2
    int recentNodeExclusion = 10;
    double loopClosure_ICPMaxError = 0.05;
    int loopClosure_ICPMinCorrespondences = 60;
    
    //capping loop closure info
    double loopClosure_minSigma = 0.02; // minimum assumed sigma (m) for loop closures
    double loopClosure_maxInformation = 400.0; //sigma 1/root(400) = 5cm max certainty

    //loop closure outlier rejection


    int nodeID = 0;

    std::vector<slam::Node> nodes;
    std::vector<slam::Edge> edges;

    public:
        bool tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp, bool* optimization_happened = nullptr);
        std::vector<slam::Node> getNodes() const {return nodes;}
        std::vector<slam::Edge> getEdges() const {return edges;}    

    private:
        bool shouldAddKeyframe(const Pose2D& pose); //compare to latest keyframe and check if we pass angle or dist threshholds
        std::vector<Edge> detectLoopClosures(const Node& queryNode);

    };


}



#endif // SLAM_POSEGRAPH_H
