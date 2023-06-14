import open3d

pcd = open3d.io.read_point_cloud('/home/pierro/Programmation/tutorial/008211.ply')

#print(pcd)

open3d.visualization.draw_geometries([pcd])
