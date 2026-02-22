#ifndef SLAM_POSEGRAPH_H
#define SLAM_POSEGRAPH_H

#include "types.h"
#include <vector>
#include <Eigen/Dense>

namespace slam {

class PoseGraph{

    double distThreshold  = 0.2;   // 20 cm
    double angleThreshold = 0.06;  // ~4 deg

    int nodeID = 0;

    std::vector<slam::Node> nodes;
    std::vector<slam::Edge> edges;

    public:
        bool tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp);
        std::vector<slam::Node> getNodes() const {return nodes;}
        std::vector<slam::Edge> getEdges() const {return edges;}    

    private:
        bool shouldAddKeyframe(const Pose2D& pose); //compare to latest keyframe and check if we pass angle or dist threshholds
};


}



#endif // SLAM_POSEGRAPH_H
