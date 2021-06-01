import open3d as o3d
import numpy as np
import copy
import os
import sys

# source = o3d.io.read_point_cloud("data/TestData/ICP/cloud_bin_0.pcd")
# target = o3d.io.read_point_cloud("data/TestData/ICP/cloud_bin_1.pcd")

source = o3d.io.read_point_cloud("data/Market2Loops/2loops.ply")
target = o3d.io.read_point_cloud("data/Market2Loops/1loopinv.ply")

threshold = 0.02
trans_init = np.asarray([[0.862, 0.011, -0.507, 0.5],
                         [-0.139, 0.967, -0.215, 0.7],
                         [0.487, 0.255, 0.835, -1.4], 
                         [0.0, 0.0, 0.0, 1.0]])

source_temp = copy.deepcopy(source)
target_temp = copy.deepcopy(target)
source_temp.paint_uniform_color([1, 0.706, 0])
target_temp.paint_uniform_color([0, 0.651, 0.929])
source_temp.transform(trans_init)
o3d.visualization.draw_geometries([source_temp, target_temp])

print("Apply point-to-point ICP")
reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)