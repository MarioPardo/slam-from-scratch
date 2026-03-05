#include "PoseGraph.h"
#include "icp_matcher.h"
#include "lidar_processor.h"

namespace slam{

bool PoseGraph::shouldAddKeyframe(const Pose2D& pose)
{
    if(nodes.empty())
        return true;

    Pose2D prevPose = nodes.back().pose;

    float dx = pose.x - prevPose.x;
    float dy = pose.y - prevPose.y;
    float dist = sqrt(dx*dx + dy*dy);

    float dtheta = pose.theta - prevPose.theta;
    if(dtheta > M_PI) dtheta -= 2*M_PI;
    if(dtheta < -M_PI) dtheta += 2*M_PI;

    return (dist > distThreshold || abs(dtheta) > angleThreshold);  
}

bool PoseGraph:: tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp)
{
    if(!shouldAddKeyframe(pose))
        return false;

    Node newNode = {nodeID, timestamp, pose, scan};
    nodeID++;

    if(!nodes.empty())
    {
        Transform2D rel_trans = slam::computePoseDelta(nodes.back().pose, pose);
        Edge newEdge = {nodes.back().id, newNode.id, MOVEMENT, rel_trans};
        edges.push_back(newEdge);
    }

    nodes.push_back(newNode);

    //added node, now let's check for loop closures
    std::vector<Edge> loop_closure_edgesthis = this->detectLoopClosures(nodes.back());
    
    for(const Edge& edge : loop_closure_edgesthis)
    {
        edges.push_back(edge);
    }

    return true;
}

std::vector<Edge> PoseGraph::detectLoopClosures(const Node& queryNode)
{

    //iterate through all nodes/ ignore most recent 10
    std::vector<Edge> loopClosureEdges = {};

    if(nodes.size() <= recentNodeExclusion)
        return {};

    
    //ok for now, must be optimized (kd trees etc) when dealing with more data
    for(int i = 0; i < nodes.size() - recentNodeExclusion; i++)
    {
        Node& candidateNode = nodes[i];

        float dist= std::hypot(candidateNode.pose.x - queryNode.pose.x, candidateNode.pose.y - queryNode.pose.y);

        if(dist > loopClosure_Radius)
            continue;

        //valid candiate, let's run ICP and check results
        Transform2D initial_guess = computePoseDelta(candidateNode.pose, queryNode.pose);

        ICPResult icpresult = alignPointClouds(scanToPointCloud(queryNode.lidar_scan),
                            scanToPointCloud(candidateNode.lidar_scan),
                            initial_guess, 80, 1e-6,0.4);

        //Add edge if loop closure found
        if(icpresult.converged && icpresult.final_error <= this->loopClosure_ICPMaxError && icpresult.correspondence_count >= this->loopClosure_ICPMinCorrespondences)
        {
            Edge loopClosureEdge = {queryNode.id, candidateNode.id, LOOP_CLOSURE, icpresult.transform};
            loopClosureEdges.push_back(loopClosureEdge);
        }
    }

    return loopClosureEdges;
}


}