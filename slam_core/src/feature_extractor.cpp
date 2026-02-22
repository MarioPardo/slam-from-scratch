#include "feature_extractor.h"
#include <iostream>
#include <algorithm>

namespace slam {

FeatureExtractor::FeatureExtractor(const SplitMergeParams& params) 
    : params_(params) {
}

void FeatureExtractor::setParams(const SplitMergeParams& params) {
    params_ = params;
}

std::vector<LineSegment> FeatureExtractor::extractLines(
    const std::vector<Eigen::Vector2d>& points) {
    
    if (points.size() < 2) {
        return std::vector<LineSegment>();
    }
    
    // Step 1: Split phase - recursively split the point sequence
    std::vector<std::pair<int, int>> segments;
    splitRecursive(points, 0, points.size() - 1, segments);
    
    // Step 2: Merge phase - merge collinear adjacent segments
    std::vector<LineSegment> lines = mergeSegments(points, segments);
    
    // Step 3: Filter out lines that don't meet minimum requirements
    std::vector<LineSegment> filtered_lines;
    for (const auto& line : lines) {
        if (line.point_indices.size() >= static_cast<size_t>(params_.min_points_per_line) &&
            line.length >= params_.min_line_length) {
            filtered_lines.push_back(line);
        }
    }
    
    return filtered_lines;
}

// gets the edge + center points of a line
std::vector<Eigen::Vector2d> getLinesMainPoints(const std::vector<slam::LineSegment>& lines) {
    std::vector<Eigen::Vector2d> points;
    points.reserve(lines.size() * 3);
    
    for (const auto& line : lines) 
    {
        points.push_back(line.start);
        points.push_back(line.midpoint());
        points.push_back(line.end);
    }
    
    return points;
}

void FeatureExtractor::splitRecursive(
    const std::vector<Eigen::Vector2d>& points,
    int start_idx, int end_idx,
    std::vector<std::pair<int, int>>& segments) {
    
    // Base case: not enough points for a line
    if (end_idx - start_idx < 1) {
        return;
    }
    
    // Fit a line to the current segment
    LineSegment line = fitLine(points, start_idx, end_idx);
    
    // Find the point with maximum distance from this line
    double max_distance;
    int farthest_idx = findFarthestPoint(points, start_idx, end_idx, line, max_distance);
    
    // If the maximum distance exceeds the threshold, split at that point
    if (max_distance > params_.split_threshold) {
        splitRecursive(points, start_idx, farthest_idx, segments);
        splitRecursive(points, farthest_idx, end_idx, segments);
    } else {
        // This segment is a good line, add it
        segments.push_back({start_idx, end_idx});
    }
}

LineSegment FeatureExtractor::fitLine(
    const std::vector<Eigen::Vector2d>& points,
    int start_idx, int end_idx) {
    
    // For now, use a simple line between endpoints
    // Could use least-squares fitting for better accuracy
    
    // Simple method: just use start and end points
    LineSegment line(points[start_idx], points[end_idx]);
    
    // Store the indices of points in this segment
    for (int i = start_idx; i <= end_idx; ++i) {
        line.point_indices.push_back(i);
    }
    
    return line;
    
    /* Alternative: Least squares fitting
    // Calculate centroid
    Eigen::Vector2d centroid = Eigen::Vector2d::Zero();
    int count = 0;
    for (int i = start_idx; i <= end_idx; ++i) {
        centroid += points[i];
        count++;
    }
    centroid /= count;
    
    // Build covariance matrix
    double xx = 0, xy = 0, yy = 0;
    for (int i = start_idx; i <= end_idx; ++i) {
        Eigen::Vector2d p = points[i] - centroid;
        xx += p.x() * p.x();
        xy += p.x() * p.y();
        yy += p.y() * p.y();
    }
    
    // Find principal direction (eigenvector of covariance matrix)
    double trace = xx + yy;
    double det = xx * yy - xy * xy;
    double eigenvalue = (trace + std::sqrt(trace * trace - 4 * det)) / 2;
    
    Eigen::Vector2d direction;
    if (std::abs(xy) > 1e-8) {
        direction = Eigen::Vector2d(eigenvalue - yy, xy);
        direction.normalize();
    } else {
        direction = (xx > yy) ? Eigen::Vector2d(1, 0) : Eigen::Vector2d(0, 1);
    }
    
    // Project first and last points onto the line
    Eigen::Vector2d p_start = points[start_idx] - centroid;
    Eigen::Vector2d p_end = points[end_idx] - centroid;
    
    double t_start = p_start.dot(direction);
    double t_end = p_end.dot(direction);
    
    if (t_start > t_end) {
        std::swap(t_start, t_end);
    }
    
    LineSegment line;
    line.start = centroid + t_start * direction;
    line.end = centroid + t_end * direction;
    line.update();
    
    return line;
    */
}

int FeatureExtractor::findFarthestPoint(
    const std::vector<Eigen::Vector2d>& points,
    int start_idx, int end_idx,
    const LineSegment& line,
    double& max_distance) {
    
    max_distance = 0.0;
    int farthest_idx = start_idx;
    
    for (int i = start_idx + 1; i < end_idx; ++i) {
        double dist = line.distanceToPoint(points[i]);
        if (dist > max_distance) {
            max_distance = dist;
            farthest_idx = i;
        }
    }
    
    return farthest_idx;
}

std::vector<LineSegment> FeatureExtractor::mergeSegments(
    const std::vector<Eigen::Vector2d>& points,
    const std::vector<std::pair<int, int>>& segments) {
    
    if (segments.empty()) {
        return std::vector<LineSegment>();
    }
    
    // Convert segment indices to LineSegments
    std::vector<LineSegment> lines;
    for (const auto& seg : segments) {
        LineSegment line = fitLine(points, seg.first, seg.second);
        lines.push_back(line);
    }
    
    // Merge adjacent collinear segments
    bool merged = true;
    while (merged) {
        merged = false;
        
        for (size_t i = 0; i < lines.size() - 1; ++i) {
            if (canMerge(lines[i], lines[i + 1])) {
                lines[i] = mergeTwo(lines[i], lines[i + 1]);
                lines.erase(lines.begin() + i + 1);
                merged = true;
                break;
            }
        }
    }
    
    return lines;
}

bool FeatureExtractor::canMerge(const LineSegment& line1, const LineSegment& line2) const {
    // Check 1: Are the directions similar?
    double angle = angleBetween(line1.direction, line2.direction);
    if (angle > params_.merge_threshold) {
        return false;
    }
    
    // Check 2: Are the endpoints close enough?
    double gap = endpointDistance(line1, line2);
    if (gap > params_.merge_distance) {
        return false;
    }
    
    return true;
}

LineSegment FeatureExtractor::mergeTwo(const LineSegment& line1, const LineSegment& line2) {
    // Create a new line from the start of line1 to the end of line2
    LineSegment merged(line1.start, line2.end);
    
    // Combine point indices
    merged.point_indices = line1.point_indices;
    merged.point_indices.insert(
        merged.point_indices.end(),
        line2.point_indices.begin(),
        line2.point_indices.end()
    );
    
    return merged;
}

double FeatureExtractor::angleBetween(const Eigen::Vector2d& v1, const Eigen::Vector2d& v2) const {
    // Ensure normalized vectors
    Eigen::Vector2d n1 = v1.normalized();
    Eigen::Vector2d n2 = v2.normalized();
    
    // Calculate angle using dot product
    double dot = std::max(-1.0, std::min(1.0, n1.dot(n2)));
    return std::acos(std::abs(dot));  // Use absolute value to ignore direction
}

double FeatureExtractor::endpointDistance(const LineSegment& line1, const LineSegment& line2) const {
    // Calculate the gap between the end of line1 and the start of line2
    return (line2.start - line1.end).norm();
}

} // namespace slam
