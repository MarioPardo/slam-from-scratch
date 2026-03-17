#ifndef SLAM_FEATURE_EXTRACTOR_H
#define SLAM_FEATURE_EXTRACTOR_H

#include "types.h"
#include <vector>
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <cmath>
#include <limits>

namespace slam {

// Represents a line segment extracted from the point cloud
struct LineSegment {
    Eigen::Vector2d start;  // Start point of the line
    Eigen::Vector2d end;    // End point of the line
    double length;          // Length of the line segment
    Eigen::Vector2d direction; // Unit direction vector
    std::vector<int> point_indices; // Indices of points that belong to this line
    
    LineSegment() : length(0.0) {}
    
    LineSegment(const Eigen::Vector2d& s, const Eigen::Vector2d& e) 
        : start(s), end(e) {
        update();
    }
    
    void update() {
        Eigen::Vector2d diff = end - start;
        length = diff.norm();
        if (length > 1e-6) {
            direction = diff / length;
        } else {
            direction = Eigen::Vector2d(1, 0);
        }
    }
    
    // Get the midpoint of the line segment
    Eigen::Vector2d midpoint() const {
        return (start + end) / 2.0;
    }
    
    // Calculate perpendicular distance from a point to the line
    double distanceToPoint(const Eigen::Vector2d& point) const {
        Eigen::Vector2d start_to_point = point - start;
        Eigen::Vector2d start_to_end = end - start;
        
        double line_length_sq = start_to_end.squaredNorm();
        if (line_length_sq < 1e-8) {
            return start_to_point.norm();
        }
        
        // Project point onto the line and find perpendicular distance
        double t = start_to_point.dot(start_to_end) / line_length_sq;
        
        // Clamp t to [0, 1] to stay within segment bounds
        t = std::max(0.0, std::min(1.0, t));
        
        Eigen::Vector2d projection = start + t * start_to_end;
        return (point - projection).norm();
    }
};

// Parameters for split-and-merge algorithm
struct SplitMergeParams {
    double split_threshold;      // Maximum distance from point to line before split (meters)
    double merge_threshold;      // Maximum angle difference for merging adjacent lines (radians)
    double merge_distance;       // Maximum gap distance for merging lines (meters)
    int min_points_per_line;     // Minimum number of points to form a valid line
    double min_line_length;      // Minimum length of a line segment (meters)
    
    SplitMergeParams() 
        : split_threshold(0.05),              // 5 cm
          merge_threshold(0.174533),          // 10 degrees in radians
          merge_distance(0.15),               // 15 cm
          min_points_per_line(5),
          min_line_length(0.1) {}             // 10 cm
};

class FeatureExtractor {
public:
    explicit FeatureExtractor(const SplitMergeParams& params = SplitMergeParams());
    
    // Main extraction function: convert point cloud to line segments
    std::vector<LineSegment> extractLines(const std::vector<Eigen::Vector2d>& points);
    
    // Set parameters
    void setParams(const SplitMergeParams& params);

    std::vector<Eigen::Vector2d> getLinesMainPoints(const std::vector<slam::LineSegment>& lines);
    
    // Get current parameters
    const SplitMergeParams& getParams() const { return params_; }

    
private:
    SplitMergeParams params_;
    
    // Split phase: recursively split point sequences into line segments
    void splitRecursive(const std::vector<Eigen::Vector2d>& points,
                       int start_idx, int end_idx,
                       std::vector<std::pair<int, int>>& segments);
    
    // Merge phase: merge adjacent collinear line segments
    std::vector<LineSegment> mergeSegments(
        const std::vector<Eigen::Vector2d>& points,
        const std::vector<std::pair<int, int>>& segments);
    
    // Fit a line to a set of points using least squares
    LineSegment fitLine(const std::vector<Eigen::Vector2d>& points,
                       int start_idx, int end_idx);
    
    // Find the point with maximum perpendicular distance from the line
    int findFarthestPoint(const std::vector<Eigen::Vector2d>& points,
                         int start_idx, int end_idx,
                         const LineSegment& line,
                         double& max_distance);
                         
    // Check if two line segments can be merged
    bool canMerge(const LineSegment& line1, const LineSegment& line2) const;
    
    // Merge two line segments into one
    LineSegment mergeTwo(const LineSegment& line1, const LineSegment& line2);
    
    // Calculate the angle between two direction vectors
    double angleBetween(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const;
    
    // Calculate distance between two line segment endpoints
    double endpointDistance(const LineSegment& line1, const LineSegment& line2) const;
};

} // namespace slam

#endif // SLAM_FEATURE_EXTRACTOR_H
