#include "../include/icp.h"


PointCloud icp::loadCoordinates(const std::string& filename){
    PointCloud cloud;

    std::ifstream file;
    float x_coord, y_coord, z_coord;

    file.open(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return {};
    }
    while (file >> x_coord >> y_coord >> z_coord){
        cloud.emplace_back(x_coord, y_coord, z_coord);
    }
    file.close();
    return cloud; 
}

/* Brute force method to finding its nearest neighbor*/
/* For all points in src, there exists a point in tgt that is closest to a point in src, has minimized distance between both points*/
std::vector<int> icp::findCorrespondenses(const PointCloud &src, const PointCloud& tgt){
    std::vector<int> correspondences;

    for(const auto& pt : src){
        // initiliazing the min_distance to a high number 
        float min_dist = std::numeric_limits<float>::max();
        // initiliazing index to closest target point 
        int min_idx = 0;

        for(size_t j = 0; j < tgt.size(); j++){
            /* tries to get the minimium euclidian distance between pt and tgt[j]. 
                || pt - tgt[j] ||^2
            */
            float dist = (pt - tgt[j]).squaredNorm();
            // if the calculated distance is less than the min_dist (initially set as a large number), it will switch to that and capture the index to be saved
            if(dist < min_dist){
                min_dist = dist;
                min_idx = j; 
            }
        }
        /* vector of tgt indexes closest to src points 
        */
        correspondences.push_back(min_idx);
    }
    return correspondences;
}

std::vector<Eigen::Vector3d> icp::computeCentroids(const PointCloud &pointcloud){
    //Initializing point
    Eigen::Vector3d centroid_pt(0,0,0);

    if(pointcloud.empty()) {
        std::cout << "Vector is empty. Returning now.." std::endl; 
        return centroid_pt;
    }

    // Adding up all of the points in the pointcloud  
    for(const auto&pt : pointcloud)
        centroid_pt += pt; 

    // taking all of the points and dividing by size 
    centroid_pt /= centroid_pt.size();

    return centroid_pt; 
}


Eigen::Matrix4f estimateTransformation(){
    std::vector<int> src = loadCoordinates("./sort1.csv");
    std::vector<int> tgt = loadCoordinates("./sort2.csv");
    std::vector<int> indexes = icp.findCorrespondenses(cloud, cloud2);

    // CENTROID POINTS
    std::vector<Eigen::Vector3d> centroid_src = computeCentroids(src);
    std::vector<Eigen::Vector3d> centroid_tgt = computeCentroids(tgt);

    // Subtracting centroids from regular points
    std::vector<Eigen::Vector3d> center_src, center_tgt;

    for(size_t i = 0; i < src.size(); i++){
        center_src.push_back(src[i] - centroid_src);
        center_tgt.push_back(tgt[i] - centroid_tgt);
    }

    for(size_t i : center_src)
        std::cout << center_src[i] << std::endl; 

}



int main()
{
    icp icp; 
    
    Eigen::Matrix4f = icp.estimateTransformation();



    return 0;
}