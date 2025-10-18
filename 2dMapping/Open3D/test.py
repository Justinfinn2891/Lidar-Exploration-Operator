import open3d as o3d
import numpy as np
import pandas as pd

# Function to load point cloud data from CSV
def load_point_cloud_from_csv(file_path):
    # Load CSV data into pandas DataFrame
    df = pd.read_csv(file_path, header=None)
    # Assume the columns are x, y, z (and possibly other data like intensity)
    points = df.iloc[:, :3].values  # Get first three columns (x, y, z)
    
    # Create Open3D PointCloud object from the points
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

# Function to preprocess the point cloud (downsampling and normal estimation)
def preprocess(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30)
    )
    return pcd_down

# Global Registration (initial alignment)
def execute_global_registration(pcd1, pcd2, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_icp(
        pcd1, pcd2, distance_threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    return result

# ICP Registration (fine alignment)
def refine_registration(pcd1, pcd2, voxel_size):
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd2, pcd1, max_correspondence_distance=voxel_size * 1.5,
        init=np.identity(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    return reg_p2p

# Main function to merge the scans
def merge_scans(scan1_path, scan2_path, voxel_size=0.02, output_path="merged_scan.pcd"):
    # Load point clouds from CSV files
    pcd1 = load_point_cloud_from_csv(scan1_path)
    pcd2 = load_point_cloud_from_csv(scan2_path)

    # Preprocess point clouds (downsample and estimate normals)
    pcd1_down = preprocess(pcd1, voxel_size)
    pcd2_down = preprocess(pcd2, voxel_size)

    # Global registration (rough alignment)
    print("Performing global registration...")
    reg_global = execute_global_registration(pcd1_down, pcd2_down, voxel_size)
    print("Global registration done.")

    # Refine the alignment using ICP (Iterative Closest Point)
    print("Refining registration with ICP...")
    reg_refined = refine_registration(pcd1_down, pcd2_down, voxel_size)
    print("ICP refinement done.")

    # Transform second scan (pcd2) based on the ICP result
    transformed_pcd2 = pcd2.transform(reg_refined.transformation)

    # Merge the two point clouds
    print("Merging point clouds...")
    combined_pcd = pcd1 + transformed_pcd2

    # Optionally remove outliers to clean the merged point cloud
    print("Removing outliers...")
    combined_clean, _ = combined_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # Save the merged and cleaned point cloud as a PCD file
    o3d.io.write_point_cloud(output_path, combined_clean)
    print(f"Saved merged point cloud to {output_path}")

    # Visualize the result
    o3d.visualization.draw_geometries([combined_clean])
    return combined_clean

# Usage example
if __name__ == "__main__":
    scan1_path = "scan1.csv"  # Path to the first scan (CSV format)
    scan2_path = "scan2.csv"  # Path to the second scan (CSV format)
    output_path = "merged_scan.pcd"  # Path to save the merged point cloud
    voxel_size = 0.02  # Adjust voxel size based on your data's resolution

    merge_scans(scan1_path, scan2_path, voxel_size, output_path)
