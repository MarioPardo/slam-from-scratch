#include "PoseGraph.h"
#include "icp_matcher.h"
#include "lidar_processor.h"
#include "pose_graph_optimizer.h"
#include <iostream>
#include "math.h"
#include <algorithm>
#include <cstdlib>
#include <limits>

namespace slam{

namespace {
bool pgChainDiagEnabled() {
    static bool enabled = (std::getenv("SLAM_PG_CHAIN_DIAG") != nullptr);
    return enabled;
}

    double kMovementSigmaX = 0.5;
    double kMovementSigmaY = 0.5;
    double kMovementSigmaTheta = 0.1;
    double kLoopToMovementInfoRatioCap = 3.0;
    double kLoopHardRejectTransResidual = 0.25;
    double kLoopHardRejectRotResidual = 0.20;
    double kLoopBestScoreRotWeight = 0.5;


double angleOf(const Eigen::Matrix2d& R) {
    return std::atan2(R(1, 0), R(0, 0));
}

double wrappedAbsAngleDiff(double a, double b) {
    double d = std::fmod(std::fabs(a - b) + M_PI, 2 * M_PI) - M_PI;
    return std::fabs(d);
}

Eigen::Vector3d movementInfoDiag() {
    Eigen::Vector3d sigma(kMovementSigmaX, kMovementSigmaY, kMovementSigmaTheta);
    return (sigma.array().inverse().square()).matrix();
}
}

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
        Eigen::Vector3d info_diag = movementInfoDiag();
        Edge newEdge = {nodes.back().id, newNode.id, MOVEMENT, rel_trans, info_diag.asDiagonal()};
        edges.push_back(newEdge);

    }

    nodes.push_back(newNode);

    //added node, now let's check for loop closures
    std::vector<Edge> loop_closure_edges= this->detectLoopClosures(nodes.back());
    for(const Edge& edge : loop_closure_edges)
    {
        edges.push_back(edge);
    }

    //if loop closure found, try optimizing graph
    if(!loop_closure_edges.empty())
    {
        std::cout<<"Loop Closure found, Optimizing!" <<std::endl;
        bool success = PoseGraphOptimizer::optimize(this->nodes, this->edges);
        if(success)
            std::cout<<"Pose Graph Optimized!" <<std::endl;
    }

    return true;
}

std::vector<Edge> PoseGraph::detectLoopClosures(const Node& queryNode)
{

    //iterate through all nodes/ ignore most recent 10
    std::vector<Edge> loopClosureEdges = {};

    //keep score for best edge
    bool hasBestLoop = false;
    Edge bestLoopEdge;
    double bestScore = std::numeric_limits<double>::infinity();
    double bestTransRes = 0.0;
    double bestRotRes = 0.0;

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

        ICPResult icpresult = alignPointClouds(scanToPointCloud(candidateNode.lidar_scan),
                            scanToPointCloud(queryNode.lidar_scan),
                            initial_guess, 100, 1e-6,0.2);


        //"angle hack" just like we do in main
        double raw_angle   = std::atan2(icpresult.transform.rotation(1, 0), icpresult.transform.rotation(0, 0));
        double fixed_angle = -raw_angle;
        Eigen::Matrix2d fixed_rotation;
        fixed_rotation << std::cos(fixed_angle), -std::sin(fixed_angle),
                            std::sin(fixed_angle),  std::cos(fixed_angle);
        slam::Transform2D fixed_transform(fixed_rotation, icpresult.transform.translation);
        icpresult.transform = fixed_transform;
        


        //Add edge if loop closure found
        if(icpresult.converged && icpresult.final_error <= this->loopClosure_ICPMaxError && icpresult.correspondence_count >= this->loopClosure_ICPMinCorrespondences)
        {
            //reject node if outlier
            Transform2D predicted = computePoseDelta(candidateNode.pose, queryNode.pose);
            double trans_res = (predicted.translation - icpresult.transform.translation).norm();
            double rot_res = wrappedAbsAngleDiff(angleOf(predicted.rotation), angleOf(icpresult.transform.rotation));

            if (trans_res > kLoopHardRejectTransResidual || rot_res > kLoopHardRejectRotResidual) 
                continue;
            

            // clamp information so we don't get too high of certainty
            double sigma = std::max(this->loopClosure_minSigma, (double)icpresult.final_error);
            double scalar_info = 1.0 / (sigma * sigma);
            if(scalar_info > this->loopClosure_maxInformation) scalar_info = this->loopClosure_maxInformation;

            Eigen::Vector3d loop_info_diag(scalar_info, scalar_info, scalar_info);
            Eigen::Vector3d movement_info_diag = movementInfoDiag();
            Eigen::Vector3d max_allowed_info_diag = movement_info_diag * kLoopToMovementInfoRatioCap;
            loop_info_diag = loop_info_diag.cwiseMin(max_allowed_info_diag);

            //keep track of best candidate node yet
            double score = trans_res + kLoopBestScoreRotWeight * rot_res;
            Edge candidateEdge = {candidateNode.id, queryNode.id, LOOP_CLOSURE, icpresult.transform, loop_info_diag.asDiagonal()};
            if (!hasBestLoop || score < bestScore) {
                hasBestLoop = true;
                bestScore = score;
                bestLoopEdge = candidateEdge;
                bestTransRes = trans_res;
                bestRotRes = rot_res;
            }
        }

    }

    if (hasBestLoop) 
        loopClosureEdges.push_back(bestLoopEdge);
  
    


    return loopClosureEdges;
}


}