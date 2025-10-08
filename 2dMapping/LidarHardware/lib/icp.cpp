#include "../include/icp.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <Eigen/Dense>

PointCloud icp::loadCoordinates(const std::string& filename) {
    PointCloud cloud;
    std::ifstream file(filename);
    float x, y, z;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    while (file >> x >> y >> z)
        cloud.emplace_back(x, y, z);

    return cloud;
}

std::vector<int> icp::findCorrespondenses(const PointCloud& src, const PointCloud& tgt) {
    std::vector<int> correspondences;

    for (const auto& pt : src) {
        float min_dist = std::numeric_limits<float>::max();
        int min_idx = 0;

        for (size_t j = 0; j < tgt.size(); ++j) {
            float dist = (pt - tgt[j]).squaredNorm();
            if (dist < min_dist) {
                min_dist = dist;
                min_idx = j;
            }
        }

        correspondences.push_back(min_idx);
    }

    return correspondences;
}

Eigen::Vector3f icp::computeCentroids(const PointCloud& pointcloud) {
    Eigen::Vector3f centroid(0, 0, 0);

    if (pointcloud.empty()) {
        std::cout << "Vector is empty. Returning now.." << std::endl;
        return centroid;
    }

    for (const auto& pt : pointcloud)
        centroid += pt;

    centroid /= pointcloud.size();
    return centroid;
}

Eigen::Matrix4f icp::estimateTransformation() {
    PointCloud src = loadCoordinates("../sort.csv");
    PointCloud tgt = loadCoordinates("../sort2.csv");

    std::vector<int> indexes = findCorrespondenses(src, tgt);

    Eigen::Vector3f centroid_src = computeCentroids(src);
    Eigen::Vector3f centroid_tgt = computeCentroids(tgt);

    std::vector<Eigen::Vector3f> center_src, center_tgt;
    for (size_t i = 0; i < src.size(); i++) {
        center_src.push_back(src[i] - centroid_src);
        center_tgt.push_back(tgt[i] - centroid_tgt);
    }

    for (const auto& p : center_src)
        std::cout << p.transpose() << std::endl;

    return Eigen::Matrix4f::Identity();
}

int main() {
    icp instance;
    Eigen::Matrix4f show = instance.estimateTransformation();
    std::cout << "Transformation matrix:\n" << show << std::endl;
    return 0;
}