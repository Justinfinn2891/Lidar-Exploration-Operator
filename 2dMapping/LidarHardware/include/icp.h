#ifndef ICP_H
#define ICP_H
#include "./Eigen/Dense"
#include <vector>
#include <iostream> 
#include <fstream>

using Point = Eigen::Vector3f;
using PointCloud = std::vector<Point>;
/* Unless there are any objections, we can do this portion in c++ rather than 
python because c++ will be faster when executing large PCD sets than python.. We
can transfer the new PCD to Open3D to run later.*/



/*GOALS:

1.) Read in coordinates (x,y,z)
2.) Find nearest neighbors 
3.) Estimate transformation using SVD
4.) Apply transformation to points
5.) Loop until convergence*/
class icp {

    public:
        // loading csv data
        PointCloud loadCoordinates(const std::string& filename);
        // Brute Force, finding minimum euclidian distance between points
        std::vector<int> findCorrespondenses(const PointCloud &src, const PointCloud& tgt);
        // Singular Value Decomposition
        Eigen::Matrix4f estimateTransform();
        // applying the rotation and transformation best found
        void applyTransform();
        Eigen::Matrix4f ICP();
        // saving points
        void saveXYZ(const std::string& filename, const PointCloud& cloud);

};

#endif //ICP_H