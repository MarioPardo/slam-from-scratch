#include "pose_graph_optimizer.h"
#include "icp_matcher.h"

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

    // --- Diagnostic: compare stored edge measurements to predicted from current poses
    {
        double sum_trans_res = 0.0;
        double sum_rot_res = 0.0;
        double max_trans_res = 0.0;
        double max_rot_res = 0.0;
        int count = 0;

        for (const Edge& edge : edges) {
            // find corresponding nodes
            auto it_from = std::find_if(nodes.begin(), nodes.end(), [&](const Node& n){ return n.id == edge.from_id; });
            auto it_to   = std::find_if(nodes.begin(), nodes.end(), [&](const Node& n){ return n.id == edge.to_id; });
            if (it_from == nodes.end() || it_to == nodes.end()) continue;

            Transform2D predicted = computePoseDelta(it_from->pose, it_to->pose);

            // stored measurement
            const Transform2D& meas = edge.transform;

            // translation residual
            Eigen::Vector2d dt = predicted.translation - meas.translation;
            double trans_res = dt.norm();

            // rotation residual (angle difference)
            double ang_pred = std::atan2(predicted.rotation(1,0), predicted.rotation(0,0));
            double ang_meas = std::atan2(meas.rotation(1,0), meas.rotation(0,0));
            double rot_res = std::fmod(std::fabs(ang_pred - ang_meas) + M_PI, 2*M_PI) - M_PI;
            rot_res = std::fabs(rot_res);

            sum_trans_res += trans_res;
            sum_rot_res += rot_res;
            max_trans_res = std::max(max_trans_res, trans_res);
            max_rot_res = std::max(max_rot_res, rot_res);
            ++count;

            if (trans_res > 0.2 || rot_res > 0.3) {
                std::cout << "[PG-DIAG] Edge " << edge.from_id << "->" << edge.to_id
                          << " | meas dx,dy,dth = (" << meas.translation.x() << "," << meas.translation.y() << "," << ang_meas
                          << ") predicted = (" << predicted.translation.x() << "," << predicted.translation.y() << "," << ang_pred
                          << ") trans_res=" << trans_res << " rot_res=" << rot_res << std::endl;
            }
        }

        if (count > 0) {
            std::cout << "[PG-DIAG] edges=" << count
                      << " avg_trans_res=" << (sum_trans_res/count)
                      << " max_trans_res=" << max_trans_res
                      << " avg_rot_res=" << (sum_rot_res/count)
                      << " max_rot_res=" << max_rot_res << std::endl;
        }
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
