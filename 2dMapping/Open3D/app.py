import numpy as np
import open3d as o3d
import pandas as pd
import subprocess
import os
import sys

def print_model(point_cloud):
    """Display the point cloud in an interactive Open3D window with coordinate frame."""
    try:
        axis_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=50.0)
        o3d.visualization.draw_geometries(
            [point_cloud, axis_frame],
            window_name="LiDAR Point Cloud",
            width=1000,
            height=800,
            point_show_normal=False
        )
    except Exception as e:
        # Handle OpenGL/display failures (e.g. headless/SSH/WSL)
        print(f"[WARNING] Open3D visualization failed: {e}")
        print("[INFO] Saving point cloud as output_pointcloud.ply instead.")
        o3d.io.write_point_cloud("output_pointcloud.ply", point_cloud)

def rgb_cloud(cartesian_points):
    """Create and display a point cloud with color mapped to normalized coordinates."""
    # Ensure correct array shape and type
    cartesian_points = np.asarray(cartesian_points, dtype=np.float64)
    if cartesian_points.ndim != 2 or cartesian_points.shape[1] != 3:
        raise ValueError(f"Expected Nx3 array, got shape {cartesian_points.shape}")

    # Create point cloud
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(cartesian_points)

    # Normalize coordinates for color mapping
    mins = cartesian_points.min(axis=0)
    maxs = cartesian_points.max(axis=0)
    normalized = (cartesian_points - mins) / (maxs - mins + 1e-8)

    # Assign RGB colors based on XYZ
    point_cloud.colors = o3d.utility.Vector3dVector(normalized)

    print("[INFO] Displaying point cloud...")
    print_model(point_cloud)

if __name__ == "__main__":
    try:
        # Optional: Run LiDAR hardware script
        # subprocess.run(["../LidarHardware/lidar"])

        csv_name = input("Please give the name of the .csv: ").strip()
        csv_path = os.path.join("../LidarHardware/src", csv_name)

        if not os.path.exists(csv_path):
            raise FileNotFoundError

        # Load CSV and ensure numeric
        lidar_df = pd.read_csv(csv_path, header=None, names=["x", "y", "z"])
        lidar_df = lidar_df.apply(pd.to_numeric, errors='coerce').dropna()

        # Convert to numpy array
        cartesian_points = lidar_df[["x", "y", "z"]].to_numpy(dtype=np.float64)

        # Debug output
        print("[DEBUG] cartesian_points shape:", cartesian_points.shape)
        print("[DEBUG] cartesian_points dtype:", cartesian_points.dtype)
        print("[DEBUG] First row:", cartesian_points[0])
        print(lidar_df.head(), "\n")

        # Display the point cloud
        rgb_cloud(cartesian_points)

    except FileNotFoundError:
        print(" Invalid CSV file. Please enter something else!")
    except Exception as e:
        print(f" Unexpected error: {e}")
        sys.exit(1)