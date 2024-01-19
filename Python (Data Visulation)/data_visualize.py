# Student name: Jashanjyot Randhawa
# Student number: 400337963
# Python version: 3.8.6

import numpy as np
import open3d as o3d
if __name__ == "__main__":
    
    lines = []
    po = 0
    
for x in range(10):
    for pt in range(32):
        lines.append([pt + po, pt + 1 + po])
    po += 32   

po = 0
do = 32

for x in range(9):
    for pt in range(32):
        lines.append([pt + po, pt + do + po])
    po += 32

pcd = o3d.io.read_point_cloud("2dx3data.xyz", format='xyz')

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
o3d.visualization.draw_geometries([line_set])
