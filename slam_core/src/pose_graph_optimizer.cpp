#include "pose_graph_optimizer.h"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>

namespace slam {

bool PoseGraphOptimizer::optimize(std::vector<Node>& nodes, const std::vector<Edge>& edges)
{
    if (nodes.empty() || edges.empty())
        return false;

    // Create the factor graph and initial values
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;

    for (const Node& node : nodes) 
    {
        double x = node.pose.x;
        double y = node.pose.y;
        double theta = node.pose.theta;
        gtsam::Pose2 pose(x, y, theta);
        // Use node.id as the key
        initial.insert(node.id, pose);
    }

    // Add an anchor / prior on node 0
    if (!nodes.empty()) 
    {
        const Node& node0 = nodes[0];
        gtsam::Pose2 prior_pose(node0.pose.x, node0.pose.y, node0.pose.theta);
       
        // Strong prior: small stddev
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas( (gtsam::Vector(3) << 1e-4, 1e-4, 1e-4).finished());
        graph.add(gtsam::PriorFactor<gtsam::Pose2>(node0.id, prior_pose, prior_noise));
    }

    //  Add a BetweenFactor<Pose2> per edge
    for (const Edge& edge : edges) {
        double dx = edge.transform.translation(0);
        double dy = edge.transform.translation(1);
        double dtheta = std::atan2(edge.transform.rotation(1, 0), edge.transform.rotation(0, 0));
        gtsam::Pose2 rel_pose(dx, dy, dtheta);

        auto info = edge.information;
        auto noise = gtsam::noiseModel::Gaussian::Information(info);

        graph.add(gtsam::BetweenFactor<gtsam::Pose2>(edge.from_id, edge.to_id, rel_pose, noise));
    }

    // Run optimizer (Levenberg-Marquardt)
    try 
    {
        gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial);
        gtsam::Values result = optimizer.optimize();

        double final_error = graph.error(result);
        constexpr double kErrorThreshold = 0.5; // Tunable error threshold
        if (!std::isfinite(final_error) || final_error > kErrorThreshold) {
            std::cerr << "[PoseGraphOptimizer] Optimization failed: final error = " << final_error << std::endl;
            return false;
        }

        // Write optimized results back into nodes
        for (Node& node : nodes) 
        {
            if (result.exists(node.id)) 
            {
                gtsam::Pose2 opt_pose = result.at<gtsam::Pose2>(node.id);
                node.pose.x = opt_pose.x();
                node.pose.y = opt_pose.y();
                node.pose.theta = opt_pose.theta();
            }
        }
        return true;

    } catch (const std::exception& e)
    { 
        std::cerr << "[PoseGraphOptimizer] Exception during optimization: " << e.what() << std::endl;
        return false; 
    }
}

} // namespace slam
