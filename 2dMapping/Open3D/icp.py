import open3d as o3d
import pandas as pandas
import numpy as np

def load_pcd_csv(file_path):
    df = pandas.read_csv(file_path, header=None)
    points = df.iloc[:, :3]. values
    pcd = o3d.geometry.PointCloud() # initlizating
    pcd.points = o3d.utility.Vector3dVector(points) #changes from .csv to .pcd for open3d
    return pcd

def preprocess(pcd, size_of_voxel):
    pcd_down = pcd.voxel_down_sample(size_of_voxel)
    pcd_down.estimate_normals(
        search_param = o3d.geometry.KDTreeSearchParamHybrid(radius=size_of_voxel * 5, max_nn = 50)
    )
    return pcd_down

def estimate_transformations(pcd_source, pcd_target, max_correspondence_distance):
    print(f"Beginning the ICP process with max correspondence distance of {max_correspondence_distance}")

    process = o3d.pipelines.registration.registration_icp(
        pcd_source, pcd_target,
        max_correspondence_distance = max_correspondence_distance, # How far the point swill go to find eachother
        init = np.identity(4), #4D matrix initial guess (Identity)
        estimation_method = o3d.pipelines.registration.TransformationEstimationPointToPlane() # for precision with surfaces
    )
    print(process)
    return process

def merge_scans(src_scans, tgt_scans, size_of_voxel):
    output_path = "merged.pcd"

    # Pipeline goal
    # Load all three scnas
    # Rough Align
    # Merge 
    # Save

    ## Load

    pcd1 = load_pcd_csv(src_scans)
    pcd2 = load_pcd_csv(tgt_scans)

    ## Rough Align

    Rotation = pcd2.get_rotation_matrix_from_xyz((np.pi / 1.75, 0, 0))
    pcd2.rotate(Rotation, center = (0,0,0))
    pcd2.translate((0,-934,-200))


    

    # Visualization (pcd1 is red, pcd2 is green (RGB)

    o3d.visualization.draw_geometries([
        pcd1.paint_uniform_color([1,0,0]),
        pcd2.paint_uniform_color([0,1,0])
    ])

    pcd1_down = preprocess(pcd1, size_of_voxel)
    pcd2_down = preprocess(pcd2, size_of_voxel)

    apply = estimate_transformations(
        pcd2_down, pcd1_down, 
        max_correspondence_distance = size_of_voxel * 2.0
    )
    pcd2.transform(apply.transformation)
    merged = pcd1 + pcd2

    # Adding more
    #pcd3_down = preprocess(pcd3_down, size_of_voxel)
    #merged_down = preprocess(merged_down, size_of_voxel)

    #apply_2 = estimate_transformations(
    #    pcd3_down, merged_down,
    #    max_correspondence_distance = size_of_voxel * 2.0
   # )

   # pcd3.transform(apply_2.transformation)
   # merged_total = merged + pcd3

    combined_clean, _ = merged.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    o3d.io.write_point_cloud(output_path, combined_clean)
    o3d.visualization.draw_geometries([combined_clean])




if __name__ == "__main__":

    # make it more automated later for multiple scans

    # add another variable for additional scan
    src_scan = "../tests/foyer.csv"
    tgt_scan = "../tests/living-room.csv"

    # For dividing 3D map into a grid of cubes (50 is 50mm)
    size_of_voxel = 50 

    merge_scans(src_scan, tgt_scan, size_of_voxel)
