// test_icp_vs_reference.cpp
// Loads captured scan pairs and compares:
//   [1] estimateTransform  vs  Eigen::umeyama  (single-round SVD)
//   [2] Full alignPointClouds from identity initial guess
//
// Usage: ./test_icp_vs_reference pair_0.json [pair_1.json ...]

#include "../src/icp_matcher.h"
#include <gtsam/3rdparty/Eigen/Eigen/Geometry>   // Eigen::umeyama

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ── JSON helpers ──────────────────────────────────────────────────────────────

// Returns the substring for "key":[...] including the outer brackets.
static std::string extractArray(const std::string& json, const std::string& key)
{
    std::string needle = "\"" + key + "\":";
    size_t pos = json.find(needle);
    if (pos == std::string::npos) return "";
    pos += needle.size();

    size_t depth = 0, start = 0;
    for (size_t i = pos; i < json.size(); ++i) {
        if (json[i] == '[') { if (depth++ == 0) start = i; }
        else if (json[i] == ']') { if (--depth == 0) return json.substr(start, i - start + 1); }
    }
    return "";
}

// Parses [[x,y],[x,y],...] into a vector of Vector2d.
static std::vector<Eigen::Vector2d> parseCloud(const std::string& arr)
{
    std::vector<Eigen::Vector2d> pts;
    const char* p = arr.c_str();
    while ((p = std::strchr(p, '[')) != nullptr) {
        ++p;
        double x, y;
        if (std::sscanf(p, "%lf,%lf", &x, &y) == 2)
            pts.push_back({x, y});
    }
    return pts;
}

struct ScanPair { std::vector<Eigen::Vector2d> source, target; };

static ScanPair loadScanPair(const std::string& path)
{
    std::ifstream f(path);
    if (!f) { std::cerr << "Cannot open: " << path << "\n"; return {}; }
    std::ostringstream ss;  ss << f.rdbuf();
    std::string json = ss.str();

    return { parseCloud(extractArray(json, "source")),
             parseCloud(extractArray(json, "target")) };
}

// ── Helpers ───────────────────────────────────────────────────────────────────

static double toDeg(double rad) { return rad * 180.0 / M_PI; }

static double angleOf(const Eigen::Matrix2d& R)
{ return std::atan2(R(1,0), R(0,0)); }

// ── Test 1: estimateTransform vs Eigen::umeyama ───────────────────────────────
// Finds correspondences at identity (no initial alignment), then compares the
// SVD step of your implementation against the Eigen reference.

static void testEstimateTransform(const ScanPair& sp)
{
    auto corrs = slam::findCorrespondencesPointToPoint(sp.source, sp.target, 0.2);
    std::cout << "  correspondences: " << corrs.size() << "\n";
    if (corrs.empty()) { std::cout << "  (no correspondences — skipping)\n"; return; }

    // ── Your implementation ──
    slam::Transform2D mine = slam::estimateTransform(corrs);

    // ── Eigen::umeyama reference (unweighted, no scaling) ──
    Eigen::MatrixXd src(2, corrs.size()), dst(2, corrs.size());
    for (size_t i = 0; i < corrs.size(); ++i) {
        src.col(i) = corrs[i].source;
        dst.col(i) = corrs[i].target;
    }
    Eigen::Matrix3d T_ref = Eigen::umeyama(src, dst, false);
    Eigen::Matrix2d R_ref = T_ref.topLeftCorner<2,2>();
    Eigen::Vector2d t_ref = T_ref.topRightCorner<2,1>();

    // ── Print comparison ──
    double angle_mine = toDeg(angleOf(mine.rotation));
    double angle_ref  = toDeg(angleOf(R_ref));
    double t_delta    = (mine.translation - t_ref).norm();

    std::cout << "  mine  angle=" << angle_mine << "°"
              << "  t=[" << mine.translation.x() << ", " << mine.translation.y() << "]\n";
    std::cout << "  ref   angle=" << angle_ref  << "°"
              << "  t=[" << t_ref.x() << ", " << t_ref.y() << "]\n";
    std::cout << "  delta angle=" << std::abs(angle_mine - angle_ref) << "°"
              << "  t_err=" << t_delta << " m\n";
}

// ── Test 2: Full ICP from identity ────────────────────────────────────────────
// Simulates what happens when the initial guess is zero (worst case — no odometry).

static void testFullICP(const ScanPair& sp)
{
    slam::Transform2D identity(Eigen::Matrix2d::Identity(), Eigen::Vector2d::Zero());
    slam::ICPResult r = slam::alignPointClouds(sp.source, sp.target, identity, 100, 1e-6, 0.2);

    std::cout << "  converged:    " << (r.converged ? "YES" : "NO") << "\n";
    std::cout << "  iterations:   " << r.iterations << "\n";
    std::cout << "  final_error:  " << r.final_error << "\n";
    std::cout << "  correspondences: " << r.correspondence_count << "\n";
    std::cout << "  angle: " << toDeg(angleOf(r.transform.rotation)) << "°"
              << "  t=[" << r.transform.translation.x() << ", " << r.transform.translation.y() << "]\n";
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cout << "Usage: " << argv[0] << " <pair_0.json> [pair_1.json ...]\n";
        return 1;
    }

    for (int i = 1; i < argc; ++i) {
        ScanPair sp = loadScanPair(argv[i]);

        std::cout << "\n==============================\n";
        std::cout << "File: " << argv[i] << "\n";
        std::cout << "  source: " << sp.source.size()
                  << " pts  target: " << sp.target.size() << " pts\n";

        std::cout << "\n[1] estimateTransform vs Eigen::umeyama (correspondences at identity)\n";
        testEstimateTransform(sp);

        std::cout << "\n[2] Full ICP from identity initial guess\n";
        testFullICP(sp);
    }

    return 0;
}
