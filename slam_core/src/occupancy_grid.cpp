#include "occupancy_grid.h"
#include <algorithm>

namespace slam {

OccupancyGrid::OccupancyGrid(double resolution, int width, int height, 
                             double origin_x, double origin_y)
    : width_(width), height_(height), resolution_(resolution),
      origin_x_(origin_x), origin_y_(origin_y) {
    
    grid_.resize(height_, std::vector<double>(width_, LOG_ODDS_PRIOR));
}

void OccupancyGrid::updateWithScan(const LidarScan& scan, const Pose2D& robot_pose) {
    if (scan.count < 2 || scan.ranges.empty()) {
        return;
    }

    int ray_count = std::min(scan.count, static_cast<int>(scan.ranges.size()));
    if (ray_count < 2) {
        return;
    }
    
    //find where robot is in grid
    int robot_grid_x, robot_grid_y;
    worldToGrid(robot_pose.x, robot_pose.y, robot_grid_x, robot_grid_y);
    
    if (!isInBounds(robot_grid_x, robot_grid_y)) {
        return;
    }
    
    double cos_theta = std::cos(robot_pose.theta);
    double sin_theta = std::sin(robot_pose.theta);
    
    double angle_increment = (scan.angle_max - scan.angle_min) / (ray_count - 1);

    for (int i = 0; i < ray_count; ++i) 
    {
        double range = scan.ranges[i];
        
        if (range < scan.range_min || range > scan.range_max ||  std::isnan(range) || std::isinf(range)) 
            continue;
        
        // Webots LiDAR ordering: ranges[0] is leftmost (angle_max), ranges[last] is rightmost (angle_min)
        double angle = scan.angle_max - i * angle_increment;
        
        //compute scan point coords in from robot POV
        double x_robot = range * std::cos(angle);
        double y_robot = range * std::sin(angle);
        
        //compute world coordinates of the scan point
        double world_scan_x = robot_pose.x + x_robot * cos_theta - y_robot * sin_theta;
        double world_y = robot_pose.y + x_robot * sin_theta + y_robot * cos_theta;
        
        //get the grid coordinates of the scan endpoint
        int endpoint_grid_x, endpoint_grid_y;
        worldToGrid(world_scan_x, world_y, endpoint_grid_x, endpoint_grid_y);
        
        if (!isInBounds(endpoint_grid_x, endpoint_grid_y)) 
            continue;
        
        //create "line" trace of this ray scan
        std::vector<std::pair<int, int>> ray_cells;
        traceLine(robot_grid_x, robot_grid_y, endpoint_grid_x, endpoint_grid_y, ray_cells);
        
        //update occupancy grid cells along the ray
        for (size_t j = 0; j < ray_cells.size(); ++j) 
        {
            int cell_x = ray_cells[j].first;
            int cell_y = ray_cells[j].second;
            
            if (j == ray_cells.size() - 1) {
                updateCell(cell_x, cell_y, LOG_ODDS_OCC);
            } else {
                updateCell(cell_x, cell_y, LOG_ODDS_FREE);
            }
        }
    }
}


void OccupancyGrid::worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const 
{
    grid_x = static_cast<int>((world_x - origin_x_) / resolution_);
    grid_y = static_cast<int>((world_y - origin_y_) / resolution_);
}

void OccupancyGrid::gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const 
{
    world_x = grid_x * resolution_ + origin_x_ + resolution_ / 2.0;
    world_y = grid_y * resolution_ + origin_y_ + resolution_ / 2.0;
}

bool OccupancyGrid::isInBounds(int grid_x, int grid_y) const {
    return grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_;
}

void OccupancyGrid::updateCell(int grid_x, int grid_y, double log_odds_delta) {
    if (!isInBounds(grid_x, grid_y)) {
        return;
    }
    
    grid_[grid_y][grid_x] += log_odds_delta;
    grid_[grid_y][grid_x] = std::clamp(grid_[grid_y][grid_x], LOG_ODDS_MIN, LOG_ODDS_MAX);
}

void OccupancyGrid::traceLine(int x0, int y0, int x1, int y1, 
                              std::vector<std::pair<int, int>>& cells) {
    cells.clear();
    
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    
    int x = x0;
    int y = y0;
    
    while (true) {
        cells.push_back({x, y});
        
        if (x == x1 && y == y1) {
            break;
        }
        
        int e2 = 2 * err;
        
        if (e2 > -dy) {
            err -= dy;
            x += sx;
        }
        
        if (e2 < dx) {
            err += dx;
            y += sy;
        }
    }
}


double OccupancyGrid::getProbability(int grid_x, int grid_y) const {
    if (!isInBounds(grid_x, grid_y)) {
        return 0.5;
    }
    
    return logOddsToProb(grid_[grid_y][grid_x]);
}

std::vector<std::pair<Point2D, double>> OccupancyGrid::getOccupiedWorldPoints(double threshold) const {
    std::vector<std::pair<Point2D, double>> occupied;
    
    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            double prob = getProbability(x, y);
            
            if (prob >= threshold) {
                Point2D world_point;
                gridToWorld(x, y, world_point.x, world_point.y);
                occupied.push_back({world_point, prob});
            }
        }
    }
    
    return occupied;
}

double OccupancyGrid::logOddsToProb(double log_odds) const {
    return 1.0 - 1.0 / (1.0 + std::exp(log_odds));
}

double OccupancyGrid::probToLogOdds(double prob) const {
    return std::log(prob / (1.0 - prob));
}

}
