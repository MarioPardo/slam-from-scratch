//NOTE Testing code made with LLM 
#include "../src/lidar_processor.h"
#include "../src/types.h"
#include <iostream>
#include <cmath>

using namespace slam;

void testSimpleScan() {
    std::cout << "\n=== Test: Simple 4-Ray Scan ===\n";
    
    LidarScan scan;
    scan.angle_min = -M_PI / 2;
    scan.angle_max = M_PI / 2;
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    scan.count = 4;
    scan.ranges = {1.0, 2.0, 1.5, 1.0};
    

    auto points = slam::scanToPointCloud(scan);
    
    std::cout << "Input: 4 rays from -90° to +90°\n";
    std::cout << "Ranges: [1.0, 2.0, 1.5, 1.0]\n";
    std::cout << "Output points (robot frame):\n";
    
    for (size_t i = 0; i < points.size(); ++i) {
        double angle = scan.angle_min + i * (scan.angle_max - scan.angle_min) / (scan.count - 1);
        std::cout << "  Point " << i << " (angle=" << angle * 180.0 / M_PI << "°): ";
        std::cout << "(" << points[i].x() << ", " << points[i].y() << ")\n";
        
        double expected_x = scan.ranges[i] * std::cos(angle);
        double expected_y = scan.ranges[i] * std::sin(angle);
        std::cout << "    Expected: (" << expected_x << ", " << expected_y << ")\n";
    }
    
    std::cout << "Expected 4 points in robot frame\n";
}

void testFilterInvalidRanges() {
    std::cout << "\n=== Test: Filter Invalid Ranges ===\n";
    
    LidarScan scan;
    scan.angle_min = 0;
    scan.angle_max = M_PI;
    scan.range_min = 0.1;
    scan.range_max = 5.0;
    scan.count = 6;
    scan.ranges = {1.0, INFINITY, 2.0, NAN, 0.05, 10.0};
    

    auto points = slam::scanToPointCloud(scan);
    
    std::cout << "Input: 6 rays with mixed valid/invalid ranges\n";
    std::cout << "Ranges: [1.0, INF, 2.0, NAN, 0.05(too close), 10.0(too far)]\n";
    std::cout << "Output: " << points.size() << " valid points\n";
    std::cout << "Expected: 2 valid points (indices 0 and 2)\n";
    
    for (size_t i = 0; i < points.size(); ++i) {
        std::cout << "  Point " << i << ": (" << points[i].x() << ", " << points[i].y() << ")\n";
    }
}

void testFullCircleScan() {
    std::cout << "\n=== Test: Full 360° Scan ===\n";
    
    LidarScan scan;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI;
    scan.range_min = 0.1;
    scan.range_max = 10.0;
    scan.count = 360;
    
    for (int i = 0; i < 360; ++i) {
        scan.ranges.push_back(2.0);
    }
    

    auto points = slam::scanToPointCloud(scan);
    
    std::cout << "Input: 360 rays, full circle, all ranges = 2.0\n";
    std::cout << "Output: " << points.size() << " points\n";
    std::cout << "Expected: 360 points forming a circle of radius 2.0\n";
    
    std::cout << "Sample points:\n";
    std::cout << "  Front (0°):   (" << points[180].x() << ", " << points[180].y() << ")\n";
    std::cout << "  Right (-90°): (" << points[90].x() << ", " << points[90].y() << ")\n";
    std::cout << "  Back (180°):  (" << points[0].x() << ", " << points[0].y() << ")\n";
    std::cout << "  Left (90°):   (" << points[270].x() << ", " << points[270].y() << ")\n";
}

int main() {
    std::cout << "========================================\n";
    std::cout << "    Scan to Point Cloud Tests\n";
    std::cout << "========================================\n";
    
    testSimpleScan();
    testFilterInvalidRanges();
    testFullCircleScan();
    
    std::cout << "\n=== All tests complete ===\n\n";
    return 0;
}
