import open3d as o3d
import numpy as np
import pandas as pd

# ----------------------------
# Load point cloud from CSV
# ----------------------------
def load_point_cloud_from_csv(file_path):
    df = pd.read_csv(file_path, header=None)
    points = df.iloc[:, :3].values  # first three columns = x, y, z
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    return pcd

# ----------------------------
# Preprocess (downsample + normals)
# ----------------------------
def preprocess(pcd, voxel_size):
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=50)
    )
    return pcd_down

# ----------------------------
# ICP Refinement (fine alignment)
# ----------------------------
def refine_registration(pcd_source, pcd_target, max_correspondence_distance):
    print(f"Running ICP refinement with max_corr_dist={max_correspondence_distance}")
    reg_p2p = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target,
        max_correspondence_distance=max_correspondence_distance,
        init=np.identity(4),
        estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
    )
    print(reg_p2p)
    return reg_p2p

# ----------------------------
# Main merge function
# ----------------------------
def merge_scans(scan1_path, scan2_path, scan3_path, voxel_size=50, output_path="merged_scan3.pcd"):
    # Load scans
    pcd1 = load_point_cloud_from_csv(scan1_path)
    pcd2 = load_point_cloud_from_csv(scan2_path)
    pcd3 = load_point_cloud_from_csv(scan3_path)

    # ---------------------------------
    # Rough manual alignment
    # ---------------------------------
    R2 = pcd2.get_rotation_matrix_from_xyz((np.pi / 1.75, 0, 0))
    pcd2.rotate(R2, center=(0, 0, 0))
    pcd2.translate((0, -934, -200))  # tweak these as needed

    # ---------------------------------
    # Rough manual alignment for scan 3
    # ---------------------------------
    R3 = pcd3.get_rotation_matrix_from_xyz((0, 0, 0))
    pcd3.rotate(R3, center=(0, 0, 0))
    pcd3.translate((0, 250, 2605))  # move further back / down if needed


    print("Visualizing before ICP (rough alignment)...")
    o3d.visualization.draw_geometries([
        pcd1.paint_uniform_color([1, 0, 0]),
        pcd2.paint_uniform_color([0, 1, 0]),
        pcd3.paint_uniform_color([0, 0, 1])
    ])

    # ---------------------------------
    # Downsample + estimate normals
    # ---------------------------------
    pcd1_down = preprocess(pcd1, voxel_size)
    pcd2_down = preprocess(pcd2, voxel_size)
    pcd3_down = preprocess(pcd3, voxel_size)

    # ---------------------------------
    # Step 1: Align scan2 to scan1
    # ---------------------------------
    reg_refined_12 = refine_registration(
        pcd2_down, pcd1_down,
        max_correspondence_distance=voxel_size * 2.0
    )
    pcd2.transform(reg_refined_12.transformation)
    merged_12 = pcd1 + pcd2

    # ---------------------------------
    # Step 2: Align scan3 to merged(1+2)
    # ---------------------------------
    merged_12_down = merged_12.voxel_down_sample(voxel_size)
    merged_12_down.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=50)
    )

    reg_refined_3 = refine_registration(
        pcd3_down, merged_12_down,
        max_correspondence_distance=voxel_size * 2.0
    )
    pcd3.transform(reg_refined_3.transformation)
    merged_all = merged_12 + pcd3

    # ---------------------------------
    # Merge and clean
    # ---------------------------------
    print("Removing outliers...")
    combined_clean, _ = merged_all.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    # ---------------------------------
    # Save + visualize
    # ---------------------------------
    o3d.io.write_point_cloud(output_path, combined_clean)
    print(f"✅ Saved merged point cloud to {output_path}")

    o3d.visualization.draw_geometries([combined_clean])
    return combined_clean

# ----------------------------
# Usage example
# ----------------------------
if __name__ == "__main__":
    scan1_path = "foyer.csv"
    scan2_path = "living-room.csv"
    scan3_path = "kitchen.csv"
    output_path = "merged_scan3.pcd"

    voxel_size = 50  # adjust based on scale (mm or cm)
    merge_scans(scan1_path, scan2_path, scan3_path, voxel_size, output_path)
