import open3d as o3d
import numpy as np

print("->正在加载点云... ")
pcd = o3d.io.read_point_cloud("/home/wanyel/contours/20220926/PointCloud_20220913092246086_mod_1.ply")
print(pcd)

print("->正在计算点云凸包...")
hull, _ = pcd.compute_convex_hull()
volume = hull.get_volume()
print(f"->点云凸包的体积为: {volume}")

hull_ls = o3d.geometry.LineSet.create_from_triangle_mesh(hull)
hull_ls.paint_uniform_color((1, 0, 0))
o3d.visualization.draw_geometries([pcd, hull_ls])