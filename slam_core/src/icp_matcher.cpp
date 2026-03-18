#include "icp_matcher.h"
#include <iostream>
#include <limits>

namespace slam {

Transform2D estimateTransform(const std::vector<CorrespondencePair>& correspondences) 
{
    //Soln R: R = V * Uᵀ where VU is from SVD of H
        //H = Sum(Yn-Y0)(Xn-X0)ᵀPn

    //Soln T: T = Y0 - R * X0

    //1: Compute centroids Y0, X0
    Eigen::Vector2d Y0 = Eigen::Vector2d::Zero();
    Eigen::Vector2d X0 = Eigen::Vector2d::Zero();
    float weightSum = 0;

    for(const CorrespondencePair& pair : correspondences){
        X0 += pair.source * pair.weight;
        Y0 += pair.target * pair.weight;
        weightSum += pair.weight;
    }

    Y0 = Y0 / weightSum;
    X0 = X0 / weightSum;


    //2: Compute covariance matrix H
    Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
    
    for(const CorrespondencePair& pair : correspondences)
    {
        Eigen::Vector2d source_centered = pair.source - X0;
        Eigen::Vector2d target_centered = pair.target - Y0;
    
        H += pair.weight * source_centered * target_centered.transpose();
    }

    //3: SVD
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);

    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    //Get Results
    Eigen::Matrix2d RotationMatrix = V*U.transpose();
    
    // Ensure we have a proper rotation (det = 1) not a reflection (det = -1)
    if (RotationMatrix.determinant() < 0) {
        // Flip the sign of the second column of V
        V(0, 1) = -V(0, 1);
        V(1, 1) = -V(1, 1);
        RotationMatrix = V * U.transpose();
    }
    
    Eigen::Vector2d TranslationVector = Y0 - RotationMatrix*X0;
    
    
    return Transform2D(RotationMatrix, TranslationVector);
}

std::vector<Eigen::Vector2d> transformPointCloud(
    const std::vector<Eigen::Vector2d>& points,
    const Transform2D& transform)
{
    std::vector<Eigen::Vector2d> transformed;
    
   for (const Eigen::Vector2d &p : points) {
        transformed.push_back(transform.rotation * p + transform.translation);
    }
    
    return transformed;
}

//TODO use KD trees
std::vector<CorrespondencePair> findCorrespondencesPointToPoint(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target,
    double max_distance)
{
    std::vector<CorrespondencePair> correspondences;
    
    for (const Eigen::Vector2d &spoint : source)
    {
        float bestdistance = std::numeric_limits<float>::infinity();
        CorrespondencePair best_pair(spoint, Eigen::Vector2d::Zero(), 0.0);
        
        for(const Eigen::Vector2d &tpoint : target)
        {
            float distance = (spoint - tpoint).norm();
            if (distance < bestdistance)
            {
                bestdistance = distance;
                double weight = 1 - (distance / max_distance);
                best_pair = CorrespondencePair(spoint, tpoint, weight);
            }
        }

        //reject based on max distance
        if (bestdistance < max_distance)
            correspondences.push_back(best_pair);
    }



    return correspondences;
}

ICPResult alignPointClouds(
    const std::vector<Eigen::Vector2d>& source,
    const std::vector<Eigen::Vector2d>& target,
    const Transform2D& initial_guess,
    int max_iterations,
    double convergence_epsilon,
    double correspondence_distance)
{
    ICPResult result;
    result.transform = initial_guess;
    result.iterations = 0;
    result.final_error = 0.0;
    result.converged = false;
    result.correspondence_count = 0;

    for (int iter = 0; iter < max_iterations; ++iter)
    {
        //1: Apply current transform
        std::vector<Eigen::Vector2d> transformedSource = transformPointCloud(source, result.transform);        

        //2: find correspondences
        std::vector<CorrespondencePair> correspondences = findCorrespondencesPointToPoint(transformedSource, target, correspondence_distance);
        result.correspondence_count = correspondences.size();
        if (correspondences.empty()) 
            break;

        //3: Estimate correction
        Transform2D correction = estimateTransform(correspondences);

        //4: Update transform
        result.transform.rotation = correction.rotation * result.transform.rotation;
        result.transform.translation = correction.rotation * result.transform.translation + correction.translation;

        //5: Compute error
        double total_error = 0.0;
        for (const CorrespondencePair& pair : correspondences) {
            Eigen::Vector2d diff = pair.source - pair.target;
            total_error += diff.squaredNorm(); 
        }
        result.final_error = total_error / correspondences.size();

        //6: Check convergence based on transformation change
        double translation_change = correction.translation.norm();
        double rotation_change = std::abs(std::atan2(correction.rotation(1, 0), correction.rotation(0, 0))); //extracts rotation angle
        double transform_change = translation_change + rotation_change;

        result.iterations = iter + 1;

        if (transform_change < convergence_epsilon) {
            result.converged = true;
            break;
        }
    }

    return result;
}

Transform2D computePoseDelta(const Pose2D& from, const Pose2D& to) {
    double dx = to.x - from.x;
    double dy = to.y - from.y;
    double dtheta = to.theta - from.theta;
    
    // Rotation from 'from' to 'to'
    Eigen::Matrix2d R;
    R << std::cos(dtheta), -std::sin(dtheta),
         std::sin(dtheta),  std::cos(dtheta);
    
    // Translation in GLOBAL frame
    Eigen::Vector2d t_global(dx, dy);
    
    // Transform translation to LOCAL frame of 'from' pose
    double cos_from = std::cos(from.theta);
    double sin_from = std::sin(from.theta);
    Eigen::Matrix2d R_from_inv;
    R_from_inv << cos_from, sin_from,
                 -sin_from, cos_from;
    
    Eigen::Vector2d t_local = R_from_inv * t_global;
    
    return Transform2D(R, t_local);
}

} // namespace slam
