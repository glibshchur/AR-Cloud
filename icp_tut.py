import open3d as o3d
import numpy as np
import copy
import os
import sys

#source = o3d.io.read_point_cloud("data/TestData/ICP/cloud_bin_0.pcd")
#target = o3d.io.read_point_cloud("data/TestData/ICP/cloud_bin_1.pcd")

source = o3d.io.read_point_cloud("data/xyz/ml1_yard4_step1.xyz")
target = o3d.io.read_point_cloud("data/xyz/ml1_yard4_step2.xyz")

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
