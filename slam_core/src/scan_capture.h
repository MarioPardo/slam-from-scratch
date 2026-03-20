#pragma once
#include "types.h"
#include <gtsam/3rdparty/Eigen/Eigen/Dense>
#include <fstream>
#include <string>
#include <vector>
#include <iostream>

namespace slam {

// Writes a scan pair to `path` as JSON:
//   {"source":[[x,y],...], "target":[[x,y],...]}
//
// Usage: gate with SLAM_CAPTURE_DIR env var (see main.cpp).
inline void saveScanPair(const std::vector<Eigen::Vector2d>& source,
                         const std::vector<Eigen::Vector2d>& target,
                         const std::string& path)
{
    std::ofstream f(path);
    if (!f) { std::cerr << "[Capture] Cannot open " << path << "\n"; return; }

    auto writeCloud = [&](const std::vector<Eigen::Vector2d>& pts) {
        f << "[";
        for (size_t i = 0; i < pts.size(); ++i) {
            if (i) f << ",";
            f << "[" << pts[i].x() << "," << pts[i].y() << "]";
        }
        f << "]";
    };

    f << "{\"source\":";
    writeCloud(source);
    f << ",\"target\":";
    writeCloud(target);
    f << "}\n";

    std::cout << "[Capture] " << path
              << "  (" << source.size() << " src, " << target.size() << " tgt pts)\n";
}

} // namespace slam
