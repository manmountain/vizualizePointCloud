import numpy as np
import open3d as o3d

sampleData = False
if sampleData:
    # Load bunny data.
    bunny = o3d.data.BunnyMesh()
    pcd_full = o3d.io.read_point_cloud(bunny.path)
    pcd_full.paint_uniform_color([0.5, 0.5, 0.5])
    pcd_full.estimate_normals()
    pcd_full.orient_normals_consistent_tangent_plane(1) # ??? why 1 ???
    # Get 1000 samples from original point cloud
    pcd_down = pcd_full.farthest_point_down_sample(1000)

    low_poly = False
    if (low_poly):
        pcd = pcd_down
    else:
        pcd = pcd_full
else:
    # Load data
    filename = "data.xyz"
    pointcloud = np.loadtxt(filename, skiprows=1, delimiter=';')
    ## Format to open3d usable object
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pointcloud)
    ##pcd.colors = o3d.utility.Vector3dVector(pointcloud[:,3:6]/255)
    ##pcd.normals = o3d.utility.Vector3dVector(pointcloud[:,6:9])
    # Add color and estimate normals for better visualization.
    pcd.paint_uniform_color([0.5, 0.5, 0.5])
    pcd.estimate_normals()
    pcd.orient_normals_consistent_tangent_plane(1)

    # Define rotation matrix for 90 degree rotation around z-axis
    R = pcd.get_rotation_matrix_from_xyz((np.pi / 2, 0, np.pi / 2))
    # Rotate the point cloud
    pcd.rotate(R, center=(0, 0, 0))


# Show the point cloud pcd
#o3d.visualization.draw_geometries([pcd], window_name="Point Cloud")

# Meshing strategy
strategy = 1
if strategy == 1:
    # Strategy 1: BPA
    # radius determination
    distances = pcd.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    radius = 3 * avg_dist
    # computing the mesh
    bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector([radius, radius * 2]))
    # decimating the mesh
    dec_mesh = bpa_mesh.simplify_quadric_decimation(100000) # ??? why 100000 ???
    # Optional
    dec_mesh.remove_degenerate_triangles()
    dec_mesh.remove_duplicated_triangles()
    dec_mesh.remove_duplicated_vertices()
    dec_mesh.remove_non_manifold_edges()
    mesh = dec_mesh
elif strategy == 2:
    # Strategy 2: Poisson reconstruction
    # computing the mesh
    poisson_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=8, width=0, scale=1.1, linear_fit=True)[0]
    # cropping
    bbox = pcd.get_axis_aligned_bounding_box()
    p_mesh_crop = poisson_mesh.crop(bbox)
    mesh = p_mesh_crop


#fill_holes = False
#if fill_holes:
#    # Dont know if I'm using this the right way ...
#    hole_size = 2
#    mesh = o3d.t.geometry.TriangleMesh.from_legacy(mesh).fill_holes(hole_size).to_legacy()

if sampleData:
    if not low_poly:
        # show both hires mesh and low poly point cloud
        pcd_down.paint_uniform_color([0.0, 1.0, 0.0])
        o3d.visualization.draw_geometries([mesh, pcd_down], window_name="3d Mesh")
    else:
        # show only mesh
        o3d.visualization.draw_geometries([mesh], window_name="3d Mesh")
else:
    # show mesh and point cloud
    pcd.paint_uniform_color([0.0, 1.0, 0.0]) # make the points in the cloud green before visualization
    o3d.visualization.draw_geometries([mesh, pcd], window_name="3d Mesh")
