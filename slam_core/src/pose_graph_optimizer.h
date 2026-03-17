#ifndef SLAM_POSE_GRAPH_OPTIMIZER_H
#define SLAM_POSE_GRAPH_OPTIMIZER_H

#include "types.h"
#include <vector>

namespace slam {

class PoseGraphOptimizer {
public:

    static bool optimize(std::vector<Node>& nodes, const std::vector<Edge>& edges);

};

} // namespace slam

#endif // SLAM_POSE_GRAPH_OPTIMIZER_H
