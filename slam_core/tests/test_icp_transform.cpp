//DISCLAIMER: Made using Claude Code to help development

#include "../src/icp_matcher.h"
#include "../src/types.h"
#include <iostream>
#include <cmath>
#include <vector>

using namespace slam;

void printTransform(const Transform2D& T, const std::string& label) {
    std::cout << "\n" << label << ":\n";
    std::cout << "Rotation matrix:\n" << T.rotation << "\n";
    std::cout << "Translation: [" << T.translation.x() << ", " << T.translation.y() << "]\n";
    
    double angle = std::atan2(T.rotation(1, 0), T.rotation(0, 0));
    std::cout << "Rotation angle: " << angle * 180.0 / M_PI << " degrees\n";
}

void testPureTranslation() {
    std::cout << "\n=== TEST 1: Pure Translation ===\n";
    
    // Create a simple square in source frame
    std::vector<Eigen::Vector2d> source_points = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(1, 1),
        Eigen::Vector2d(0, 1)
    };
    
    // Apply known translation: dx = 2.0, dy = 3.0
    Eigen::Vector2d known_translation(2.0, 3.0);
    std::vector<Eigen::Vector2d> target_points;
    for (const auto& p : source_points) {
        target_points.push_back(p + known_translation);
    }
    
    // Build correspondences
    std::vector<CorrespondencePair> correspondences;
    for (size_t i = 0; i < source_points.size(); ++i) {
        correspondences.emplace_back(source_points[i], target_points[i], 1.0);
    }
    
    // Estimate transformation
    Transform2D estimated = estimateTransform(correspondences);
    
    printTransform(estimated, "Estimated Transform");
    std::cout << "Expected translation: [" << known_translation.x() 
              << ", " << known_translation.y() << "]\n";
    std::cout << "Expected rotation: 0 degrees\n";
}

void testPureRotation() {
    std::cout << "\n=== TEST 2: Pure Rotation (45 degrees) ===\n";
    
    // Create points centered at origin
    std::vector<Eigen::Vector2d> source_points = {
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(0, 1),
        Eigen::Vector2d(-1, 0),
        Eigen::Vector2d(0, -1)
    };
    
    // Apply 45 degree rotation
    double angle = M_PI / 4.0;  // 45 degrees
    Eigen::Matrix2d known_rotation;
    known_rotation << std::cos(angle), -std::sin(angle),
                      std::sin(angle),  std::cos(angle);
    
    std::vector<Eigen::Vector2d> target_points;
    for (const auto& p : source_points) {
        target_points.push_back(known_rotation * p);
    }
    
    // Build correspondences
    std::vector<CorrespondencePair> correspondences;
    for (size_t i = 0; i < source_points.size(); ++i) {
        correspondences.emplace_back(source_points[i], target_points[i], 1.0);
    }
    
    // Estimate transformation
    Transform2D estimated = estimateTransform(correspondences);
    
    printTransform(estimated, "Estimated Transform");
    std::cout << "Expected rotation: 45 degrees\n";
    std::cout << "Expected translation: [0, 0]\n";
}

void testCombinedTransform() {
    std::cout << "\n=== TEST 3: Combined Rotation + Translation ===\n";
    
    // Create a triangle
    std::vector<Eigen::Vector2d> source_points = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(2, 0),
        Eigen::Vector2d(1, 1.5)
    };
    
    // Apply 30 degree rotation + translation (1.5, -2.0)
    double angle = M_PI / 6.0;  // 30 degrees
    Eigen::Matrix2d known_rotation;
    known_rotation << std::cos(angle), -std::sin(angle),
                      std::sin(angle),  std::cos(angle);
    Eigen::Vector2d known_translation(1.5, -2.0);
    
    std::vector<Eigen::Vector2d> target_points;
    for (const auto& p : source_points) {
        target_points.push_back(known_rotation * p + known_translation);
    }
    
    // Build correspondences
    std::vector<CorrespondencePair> correspondences;
    for (size_t i = 0; i < source_points.size(); ++i) {
        correspondences.emplace_back(source_points[i], target_points[i], 1.0);
    }
    
    // Estimate transformation
    Transform2D estimated = estimateTransform(correspondences);
    
    printTransform(estimated, "Estimated Transform");
    std::cout << "Expected rotation: 30 degrees\n";
    std::cout << "Expected translation: [" << known_translation.x() 
              << ", " << known_translation.y() << "]\n";
}

void testWeightedCorrespondences() {
    std::cout << "\n=== TEST 4: Weighted Correspondences ===\n";
    
    // Points with varying weights (simulating confidence)
    std::vector<Eigen::Vector2d> source_points = {
        Eigen::Vector2d(0, 0),
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(2, 0)
    };
    
    Eigen::Vector2d translation(1.0, 0.5);
    
    // Create correspondences with different weights
    std::vector<CorrespondencePair> correspondences;
    correspondences.emplace_back(source_points[0], source_points[0] + translation, 1.0);
    correspondences.emplace_back(source_points[1], source_points[1] + translation, 2.0);  // Higher weight
    correspondences.emplace_back(source_points[2], source_points[2] + translation, 0.5);  // Lower weight
    
    Transform2D estimated = estimateTransform(correspondences);
    
    printTransform(estimated, "Estimated Transform (Weighted)");
    std::cout << "Expected translation: [1.0, 0.5] (regardless of weights for pure translation)\n";
}

// Full ICP test: pure rotation alignment using alignPointClouds
void testICPPureRotationAlignment() {
    std::cout << "\n=== TEST 5: ICP Pure Rotation Alignment (45 degrees) ===\n";

    // Source: points on a unit circle axes
    std::vector<Eigen::Vector2d> source_points = {
        Eigen::Vector2d(1, 0),
        Eigen::Vector2d(0, 1),
        Eigen::Vector2d(-1, 0),
        Eigen::Vector2d(0, -1)
    };

    // Apply known 45 degree rotation to get target
    double angle = M_PI / 4.0;  // 45 degrees
    Eigen::Matrix2d known_rotation;
    known_rotation << std::cos(angle), -std::sin(angle),
                      std::sin(angle),  std::cos(angle);

    std::vector<Eigen::Vector2d> target_points;
    for (const auto& p : source_points) {
        target_points.push_back(known_rotation * p);
    }

    // Initial guess: 38 degree rotation (close to true 45 deg), zero translation
    Transform2D initial_guess;
    {
        double init_angle = 38.0 * M_PI / 180.0;
        Eigen::Matrix2d R_init;
        R_init << std::cos(init_angle), -std::sin(init_angle),
                  std::sin(init_angle),  std::cos(init_angle);
        initial_guess.rotation = R_init;
        initial_guess.translation = Eigen::Vector2d::Zero();
    }

    ICPResult result = alignPointClouds(
        source_points,
        target_points,
        initial_guess,
        /*max_iterations=*/50,
        /*convergence_epsilon=*/1e-6,
        /*correspondence_distance=*/2.0
    );

    printTransform(result.transform, "ICP Estimated Transform");
    std::cout << "Expected rotation: 45 degrees, translation ~ [0, 0]\n";
    std::cout << "ICP converged: " << (result.converged ? "true" : "false")
              << ", iterations: " << result.iterations
              << ", final_error: " << result.final_error
              << ", correspondences: " << result.correspondence_count << "\n";
}

int main() {
    std::cout << "======================================\n";
    std::cout << "  ICP Transform Estimation Tests\n";
    std::cout << "======================================\n";
    
    testPureTranslation();
    testPureRotation();
    testCombinedTransform();
    testWeightedCorrespondences();
    testICPPureRotationAlignment();
    
    std::cout << "\n=== All tests complete ===\n\n";
    return 0;
}
