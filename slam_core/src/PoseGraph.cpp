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

    double kMovementSigmaX = 0.08;
    double kMovementSigmaY = 0.08;
    double kMovementSigmaTheta = 0.05;
    double kMovementSigmaMaxXY = 0.8;
    double kMovementSigmaMaxTheta = 0.7;
    double kMovementTransResidualScale = 2.0;
    double kMovementRotResidualScale = 1.5;
    double kLoopToMovementInfoRatioCap = 3.0;
    double kLoopHardRejectTransResidual = 0.25;
    double kLoopHardRejectRotResidual = 0.5;
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

Eigen::Vector3d movementInfoDiagFromResidual(const Transform2D& icp_rel, const Transform2D& odom_rel) {
    double trans_res = (icp_rel.translation - odom_rel.translation).norm();
    double rot_res = wrappedAbsAngleDiff(angleOf(icp_rel.rotation), angleOf(odom_rel.rotation));

    double sigma_x = std::clamp(kMovementSigmaX + kMovementTransResidualScale * trans_res,kMovementSigmaX,kMovementSigmaMaxXY);
    double sigma_y = std::clamp(kMovementSigmaY + kMovementTransResidualScale * trans_res,kMovementSigmaY,kMovementSigmaMaxXY);
    double sigma_theta = std::clamp(kMovementSigmaTheta + kMovementRotResidualScale * rot_res,kMovementSigmaTheta,kMovementSigmaMaxTheta);

    Eigen::Vector3d sigma(sigma_x, sigma_y, sigma_theta);
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

    return (dist > minDistNewKeyframe || fabs(dtheta) > minAngleNewKeyframe);  
}

bool PoseGraph:: tryAddKeyframe(const Pose2D& pose, const LidarScan& scan, double timestamp, const Transform2D& odom_rel_transform, bool* optimization_happened)
{
    if (optimization_happened) *optimization_happened = false;

    if(!shouldAddKeyframe(pose))
        return false;

    Node newNode = {nodeID, timestamp, pose, scan};
    nodeID++;

    if(!nodes.empty())
    {
        Transform2D rel_trans = slam::computePoseDelta(nodes.back().pose, pose);
        Eigen::Vector3d info_diag = movementInfoDiagFromResidual(rel_trans, odom_rel_transform);
        Edge newEdge = {nodes.back().id, newNode.id, MOVEMENT, rel_trans, info_diag.asDiagonal()};
        edges.push_back(newEdge);
    }

    nodes.push_back(newNode);
    keyframesSinceLastLoopClosure++;
    keyframesSinceLastOptimization++;

    std::vector<Edge> loop_closure_edges;
    if (keyframesSinceLastLoopClosure >= minKeyframesBetweenLoopClosures)
        loop_closure_edges = this->detectLoopClosures(nodes.back());
    
    for(const Edge& edge : loop_closure_edges)
    {
        edges.push_back(edge);
    }

    if (!loop_closure_edges.empty()) 
        keyframesSinceLastLoopClosure = 0;

    //if loop closure found and optimization cooldown passed, try optimizing graph
    if(!loop_closure_edges.empty() && keyframesSinceLastOptimization >= minKeyframesBetweenOptimizations)
    {
        std::cout<<"Loop Closure found, Optimizing!" <<std::endl;
        bool success = PoseGraphOptimizer::optimize(this->nodes, this->edges);
        if(success) 
        {
            optimized_projected_scans_world = projectNodeScansToWorldFrame(nodes);
            std::cout<<"Pose Graph Optimized!" <<std::endl;
            keyframesSinceLastOptimization = 0;
            if (optimization_happened) *optimization_happened = true;
        }
    }

    return true;
}

std::vector<Edge> PoseGraph::detectLoopClosures(const Node& queryNode)
{

    //iterate through all nodes/ ignore most recent 10
    std::vector<Edge> loopClosureEdges = {};

    if(nodes.size() <= recentNodeExclusion)
        return {};

    //use only the closest candidates
    std::vector<std::pair<int, double>> candidates;
    candidates.reserve(nodes.size());

    for(int i = 0; i < static_cast<int>(nodes.size()); i++)
    {
        Node& candidateNode = nodes[i];

        if (candidateNode.id >= queryNode.id)
            continue;
        if ((queryNode.id - candidateNode.id) < recentNodeExclusion)
            continue;

        float dist= std::hypot(candidateNode.pose.x - queryNode.pose.x, candidateNode.pose.y - queryNode.pose.y);

        if(dist > maxDistLoopClosure)
            continue;

        candidates.push_back({i, dist});
    }

    std::sort(candidates.begin(), candidates.end(),
              [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                  return a.second < b.second;
              });

    //trim candidates
    if (candidates.size() > (maxLoopClosureCandidates)) 
        candidates.resize(maxLoopClosureCandidates);
    

    // run ICP only on nearest loop-closure candidates
    for(const auto& candidate : candidates)
    {
        Node& candidateNode = nodes[candidate.first];

        //valid candiate, let's run ICP and check results
        Transform2D initial_guess = computePoseDelta(candidateNode.pose, queryNode.pose);

        ICPResult icpresult = alignPointClouds(scanToPointCloudRobotFrame(queryNode.lidar_scan),
                            scanToPointCloudRobotFrame(candidateNode.lidar_scan),
                            initial_guess, 100, 1e-6, loopClosure_ICPCorrespondenceDistance);


        //Add edge if loop closure found
        if(icpresult.converged && icpresult.final_error <= this->loopClosure_ICPMaxError && icpresult.correspondence_count >= this->loopClosure_ICPMinCorrespondences)
        {
            std::cout<<" LC Edge found! !" <<std::endl;

            // clamp information so we don't get too high of certainty
            double sigma = std::max(this->loopClosure_minSigma, (double)icpresult.final_error);
            double scalar_info = 1.0 / (sigma * sigma);
            if(scalar_info > this->loopClosure_maxInformation) scalar_info = this->loopClosure_maxInformation;

            Eigen::Vector3d loop_info_diag(scalar_info, scalar_info, scalar_info);
            Eigen::Vector3d movement_info_diag = movementInfoDiag();
            Eigen::Vector3d max_allowed_info_diag = movement_info_diag * kLoopToMovementInfoRatioCap;
            loop_info_diag = loop_info_diag.cwiseMin(max_allowed_info_diag);

    
            Transform2D predicted = computePoseDelta(candidateNode.pose, queryNode.pose);
            
            double trans_res = (predicted.translation - icpresult.transform.translation).norm();
            double rot_res = wrappedAbsAngleDiff(angleOf(predicted.rotation), angleOf(icpresult.transform.rotation));
            if (rot_res > kLoopHardRejectRotResidual) {
                std::cout << " LC Edge rejected (rotation mismatch): rot=" << rot_res << std::endl;
                continue;
            }

            Edge candidateEdge = {candidateNode.id, queryNode.id, LOOP_CLOSURE, icpresult.transform, loop_info_diag.asDiagonal()};
            loopClosureEdges.push_back(candidateEdge);

            return loopClosureEdges;
        }
    }
    return loopClosureEdges;
}



}