import numpy as np
import open3d as o3d
import pandas as pd
import os, sys

def rgb_cloud(points):
    points = np.asarray(points, float)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    mins, maxs = points.min(0), points.max(0)
    norm = (points - mins) / (maxs - mins + 1e-8)
    pcd.colors = o3d.utility.Vector3dVector(norm)
    return pcd

def interactive_crop(pcd):

    app = o3d.visualization.gui.Application.instance
    app.initialize()
    vis = o3d.visualization.O3DVisualizer("Manual Crop Box", 1024, 768)
    vis.show_settings = True
    vis.add_geometry("pcd", pcd)
    app.add_window(vis)
    app.run()       
    return pcd       

if __name__ == "__main__":
    try:
        csv_name = "cslab.csv"
        csv_path = os.path.join("../tests", csv_name)
        df = pd.read_csv(csv_path, header=None, names=["x", "y", "z"]).dropna()
        pts = df.to_numpy(float)
        pcd = rgb_cloud(pts)

        o3d.visualization.draw([pcd], title="Original Cloud")
        interactive_crop(pcd)

    except Exception as e:
        print(f"❌ Error: {e}")
        sys.exit(1)
