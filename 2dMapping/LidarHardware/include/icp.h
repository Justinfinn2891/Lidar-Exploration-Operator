#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>
#include "kdtree.h"

using Point3D = Eigen::Vector3f;
using PointCloud = std::vector<Point3D>;

class icp {
public:
    icp() = default;

    // --- Core pipeline ---
    Eigen::Matrix4f estimateTransformation();
    PointCloud transformPointCloud(const PointCloud& cloud, const Eigen::Matrix4f& transform);
    PointCloud mergeAndDownsample(const PointCloud& a, const PointCloud& b, float voxel_size);

    // --- I/O ---
    PointCloud loadCoordinates(const std::string& filename);
    void saveCoordinates(const std::string& filename, const PointCloud& cloud);

private:
    // --- ICP internals ---
    std::vector<int> findCorrespondenses(const PointCloud& src, const PointCloud& tgt,
                                         float max_distance = 0.1f, bool use_mutual = false);
    Eigen::Vector3f computeCentroids(const PointCloud& cloud);
};
