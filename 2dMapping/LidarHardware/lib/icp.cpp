#include "../include/kdtree.h"
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <chrono>

// ----------------------------
// Parameters
// ----------------------------
const float VOXEL_SIZE = 1.0f;               // 5 cm voxel downsampling
const float MAX_CORRESPONDENCE_DIST = 10000;  // 1 meter max correspondence
const int MAX_ITERATIONS = 100;
const float TOLERANCE = 1e-5f;

// ----------------------------
// Type aliases
// ----------------------------
using PointCloud = std::vector<Eigen::Vector3f>;

// ----------------------------
// Voxel hashing for fast downsampling
// ----------------------------
struct VoxelKey {
    int x, y, z;
    bool operator==(const VoxelKey& o) const { return x==o.x && y==o.y && z==o.z; }
};
struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const noexcept {
        return static_cast<size_t>((uint64_t)k.x*73856093u ^ (uint64_t)k.y*19349663u ^ (uint64_t)k.z*83492791u);
    }
};

// ----------------------------
// Load & save CSV
// ----------------------------
PointCloud loadCSV(const std::string& filename)
{
    PointCloud cloud;
    std::ifstream file(filename);
    if (!file.is_open()) { std::cerr << "Failed to open " << filename << std::endl; return cloud; }

    std::string line;
    cloud.reserve(100000);
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        float x,y,z; char c1,c2;
        if (ss >> x >> c1 >> y >> c2 >> z)
            cloud.push_back(Eigen::Vector3f(x,y,z));
    }
    return cloud;
}

void saveCSV(const std::string& filename, const PointCloud& cloud)
{
    std::ofstream file(filename);
    for (const auto& pt : cloud)
        file << pt.x() << "," << pt.y() << "," << pt.z() << "\n";
}

// ----------------------------
// Voxel downsampling
// ----------------------------
PointCloud voxelDownsample(const PointCloud& cloud, float voxel_size)
{
    std::unordered_map<VoxelKey, Eigen::Vector3f, VoxelKeyHash> voxels;
    voxels.reserve(cloud.size()/4+1);
    for (const auto& pt : cloud) {
        int vx = static_cast<int>(std::floor(pt.x()/voxel_size));
        int vy = static_cast<int>(std::floor(pt.y()/voxel_size));
        int vz = static_cast<int>(std::floor(pt.z()/voxel_size));
        VoxelKey key{vx,vy,vz};
        if (voxels.find(key) == voxels.end()) voxels.emplace(key, pt);
    }
    PointCloud downsampled; downsampled.reserve(voxels.size());
    for (auto& kv : voxels) downsampled.push_back(kv.second);
    return downsampled;
}

// ----------------------------
// Centroid
// ----------------------------
Eigen::Vector3f computeCentroid(const PointCloud& cloud)
{
    Eigen::Vector3f c(0,0,0);
    for (const auto& pt : cloud) c += pt;
    return c / static_cast<float>(cloud.size());
}

// ----------------------------
// Transform helpers
// ----------------------------
Eigen::Matrix4f makeTransform(float tx, float ty, float tz,
                              float roll, float pitch, float yaw)
{
    Eigen::AngleAxisf r(roll,   Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf p(pitch,  Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf y(yaw,    Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f R = (y*p*r).toRotationMatrix();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = R;
    T(0,3) = tx; T(1,3) = ty; T(2,3) = tz;
    return T;
}

void applyTransform(PointCloud& cloud, const Eigen::Matrix4f& T)
{
    Eigen::Matrix3f R = T.block<3,3>(0,0);
    Eigen::Vector3f t = T.block<3,1>(0,3);
    for (auto& p : cloud) p = R*p + t;
}

// ----------------------------
// ICP
// ----------------------------
Eigen::Matrix4f runICP(PointCloud& src, const PointCloud& tgt)
{
    src = voxelDownsample(src, VOXEL_SIZE);
    PointCloud tgt_down = voxelDownsample(tgt, VOXEL_SIZE);
    KDTree tgt_tree(tgt_down);
    Eigen::Matrix4f total = Eigen::Matrix4f::Identity();
    float prev_error = std::numeric_limits<float>::max();
    std::vector<int> corr; corr.reserve(src.size());

    for (int iter=0; iter<MAX_ITERATIONS; ++iter)
    {
        corr.clear();
        for (auto& pt : src) corr.push_back(tgt_tree.nearestNeighbor(pt, MAX_CORRESPONDENCE_DIST));

        PointCloud src_valid, tgt_valid;
        for (size_t i=0;i<src.size();++i)
            if (corr[i]!=-1) { src_valid.push_back(src[i]); tgt_valid.push_back(tgt_down[corr[i]]); }


        Eigen::Vector3f c_src = computeCentroid(src_valid);
        Eigen::Vector3f c_tgt = computeCentroid(tgt_valid);

        Eigen::Matrix3f H = Eigen::Matrix3f::Zero();
        for (size_t i=0;i<src_valid.size();++i)
            H += (src_valid[i]-c_src)*(tgt_valid[i]-c_tgt).transpose();

        Eigen::JacobiSVD<Eigen::Matrix3f> svd(H, Eigen::ComputeFullU|Eigen::ComputeFullV);
        Eigen::Matrix3f R = svd.matrixV()*svd.matrixU().transpose();
        if (R.determinant()<0) R.col(2)*=-1;
        Eigen::Vector3f t = c_tgt - R*c_src;

        Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
        T.block<3,3>(0,0) = R;
        T.block<3,1>(0,3) = t;

        for (auto& pt : src) pt = R*pt+t;
        total = T*total;

        float err=0; for (size_t i=0;i<src_valid.size();++i) err+=(src_valid[i]-tgt_valid[i]).squaredNorm();
        err /= static_cast<float>(src_valid.size());
        std::cout << "ICP iter " << iter << " | mse " << err << " | corr " << src_valid.size() << std::endl;
        if (std::abs(prev_error-err)<TOLERANCE) break;
        prev_error = err;
    }

    return total;
}

// ----------------------------
// Merge scans
// ----------------------------
PointCloud mergeScans(const std::vector<std::string>& files,
                      const std::vector<Eigen::Matrix4f>& preTransforms,
                      const std::vector<bool>& flipScan)
{
    if (files.size()!=preTransforms.size() || files.size()!=flipScan.size()) {
        std::cerr << "Mismatch in input sizes!" << std::endl;
        return {};
    }

    PointCloud global_map = loadCSV(files[0]);
    for (size_t i=1;i<files.size();++i)
    {
        std::cout << "Merging scan " << i+1 << "/" << files.size() << std::endl;
        PointCloud scan = loadCSV(files[i]);

        // Apply pre-transform
        applyTransform(scan, preTransforms[i]);

        // Flip vertically if requested (around X-axis)
    if (i == 1) {
        // Rotate horizontally: original 90° + extra 15°
        float extra_yaw = 15.0f * M_PI / 180.0f; // convert degrees to radians
        // 3rd is 
        Eigen::Matrix4f horizontalRot = makeTransform(0, -1600, 2100, 360.4, 0 , 0);
        applyTransform(scan, horizontalRot);
    }


        // Refine with ICP
        runICP(scan, global_map);

        // Merge and downsample
        global_map.insert(global_map.end(), scan.begin(), scan.end());
        global_map = voxelDownsample(global_map, VOXEL_SIZE);
    }

    return global_map;
}

// ----------------------------
// Main
// ----------------------------
int main()
{
    std::vector<std::string> scans = {"../src.csv", "../target.csv"};
    std::vector<Eigen::Matrix4f> preTransforms = {
        Eigen::Matrix4f::Identity(),
        makeTransform(3,0,1000.0f, 0,0,0)// 3.048 rough 5ft forward, 1ft lower
    };
    std::vector<bool> flipScan = {false, true}; // flip second scan vertically

    PointCloud merged = mergeScans(scans, preTransforms, flipScan);
    saveCSV("../src/merged_scene.csv", merged);
    std::cout << "Merged scene saved to ../src/merged_scene.csv" << std::endl;

    return 0;
}
