#pragma once

#include "types.h"
#include <vector>
#include <cmath>
#include <limits>

namespace slam {

class OccupancyGrid {
public:
    OccupancyGrid(double resolution, int width, int height, double origin_x, double origin_y);
    
    void updateWithScan(const LidarScan& scan, const Pose2D& robot_pose);
    
    double getProbability(int grid_x, int grid_y) const;
    
    std::vector<std::pair<Point2D, double>> getOccupiedWorldPoints(double threshold = 0.5) const;
    
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }
    
private:
    void worldToGrid(double world_x, double world_y, int& grid_x, int& grid_y) const;
    void gridToWorld(int grid_x, int grid_y, double& world_x, double& world_y) const;
    
    bool isInBounds(int grid_x, int grid_y) const;
    
    void updateCell(int grid_x, int grid_y, double log_odds_delta);
    
    void traceLine(int x0, int y0, int x1, int y1, std::vector<std::pair<int, int>>& cells);

    double logOddsToProb(double log_odds) const;
    double probToLogOdds(double prob) const;
    
    int width_;
    int height_;
    double resolution_;
    double origin_x_;
    double origin_y_;
    
    std::vector<std::vector<double>> grid_;
    
    static constexpr double LOG_ODDS_OCC = 0.85;
    static constexpr double LOG_ODDS_FREE = -0.41;
    static constexpr double LOG_ODDS_MIN = -5.0;
    static constexpr double LOG_ODDS_MAX = 5.0;
    static constexpr double LOG_ODDS_PRIOR = 0.0;
};

}
