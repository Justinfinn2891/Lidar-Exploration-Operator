#include "../include/icp.h"
#include <iostream>
#include <fstream>
#include <limits>
#include <Eigen/Dense>

PointCloud icp::loadCoordinates(const std::string& filename) {
    PointCloud cloud;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string item;
        float coords[3];
        int i = 0;

        while (std::getline(ss, item, ',') && i < 3) {
            try {
                coords[i] = std::stof(item);
            } catch (...) {
                std::cerr << "Invalid number in file: " << item << std::endl;
                break;
            }
            i++;
        }
        if (i == 3) {
            cloud.emplace_back(coords[0], coords[1], coords[2]);
            std::cout << coords[0] << ", " << coords[1] << ", " << coords[2] << std::endl;
        }
    }

    return cloud;
}

void icp::saveCoordinates(const std::string& filename, const PointCloud& cloud) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error saving file: " << filename << std::endl;
        return;
    }

    for (const auto& pt : cloud)
        file << pt.x() << "," << pt.y() << "," << pt.z() << "\n";

    file.close();
}


std::vector<int> icp::findCorrespondenses(const PointCloud& src, const PointCloud& tgt) {
    
    std::vector<int> correspondences;
    correspondences.reserve(src.size());

    KDTree kdtree(tgt);

    for (const auto& pt : src) {
        int nearest_idx = kdtree.nearest(pt);
        correspondences.push_back(nearest_idx);
    }

    // Optional debug print
    for (size_t i = 0; i < correspondences.size(); ++i) {
        std::cout << "src[" << i << "] = " << src[i].transpose()
                  << " matched with tgt[" << correspondences[i] << "] = " << tgt[correspondences[i]].transpose() << '\n';
    }

    return correspondences;

    /*
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

    for (size_t i = 0; i < correspondences.size(); ++i) {
    std::cout << "src[" << i << "] = " << src[i].transpose()
              << " matched with tgt[" << correspondences[i] << "] = " << tgt[correspondences[i]].transpose() << '\n';
}

    return correspondences;

    */
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
    PointCloud src = loadCoordinates("../final2.csv");
    PointCloud tgt = loadCoordinates("../final3.csv");

    Eigen::Matrix4f total_transform = Eigen::Matrix4f::Identity();

    const int max_iterations = 5;
    const float tolerance = 1e-5;
    float prev_error = std::numeric_limits<float>::max();

    for (int iter = 0; iter < max_iterations; ++iter) {
        std::cout << "ITERATION: " << iter << std::endl; 
        std::vector<int> indexes = findCorrespondenses(src, tgt);
        std::cout << "Tests" << std::endl;
        Eigen::Vector3f centroid_src = computeCentroids(src);
        Eigen::Vector3f centroid_tgt = computeCentroids(tgt);

        std::vector<Eigen::Vector3f> center_src, center_tgt;
        for (size_t i = 0; i < src.size(); i++) {
            center_src.push_back(src[i] - centroid_src);
            center_tgt.push_back(tgt[indexes[i]] - centroid_tgt);
        }

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        for (size_t i = 0; i < center_src.size(); ++i)
            H += center_src[i] * center_tgt[i].transpose();

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV() * svd.matrixU().transpose();

        if (R.determinant() < 0) {
            Eigen::Matrix3f V = svd.matrixV();
            V.col(2) *= -1;
            R = V * svd.matrixU().transpose();
        }

        Eigen::Vector3f t = centroid_tgt - R * centroid_src;

        Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
        transformation.block<3,3>(0,0) = R;
        transformation.block<3,1>(0,3) = t;

        // Update total transformation
        total_transform = transformation * total_transform;

        // Apply transformation to src
        for (auto& pt : src)
            pt = R * pt + t;

        // Compute mean square error
        float mean_error = 0.0f;
        for (size_t i = 0; i < src.size(); ++i)
            mean_error += (src[i] - tgt[indexes[i]]).squaredNorm();
        mean_error /= src.size();

        if (std::abs(prev_error - mean_error) < tolerance)
            break;

        prev_error = mean_error;
    }

    return total_transform;
}

PointCloud icp::transformPointCloud(const PointCloud& cloud, const Eigen::Matrix4f& transform) {
    PointCloud transformed;

    for (const auto& pt : cloud) {
        Eigen::Vector4f homogenous_pt(pt.x(), pt.y(), pt.z(), 1.0f);  // Convert to 4D
        Eigen::Vector4f transformed_pt = transform * homogenous_pt;   // Apply transformation
        transformed.emplace_back(transformed_pt.head<3>());           // Convert back to 3D
    }

    return transformed;
}

int main() {
    icp instance;

    Eigen::Matrix4f transform = instance.estimateTransformation();
    PointCloud src = instance.loadCoordinates("../final2.csv");
    PointCloud transformed = instance.transformPointCloud(src, transform);
    instance.saveCoordinates("../aligned.csv", transformed);

    std::cout << "Transformation matrix:\n" << transform << std::endl;
    return 0;

}
