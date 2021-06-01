import open3d as o3d
import numpy as np
import copy
import time
import os
import sys

def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])

source = o3d.io.read_point_cloud("data/LiDAR/LiDAR Loops.ply")
target = o3d.io.read_point_cloud("data/Market/iPadPro.ply")
threshold = 0.02
trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], 
                         [0.0, 1.0, 0.0, 0.0],
                         [0.0, 0.1, 1.0, 0.0], 
                         [0.0, 0.0, 0.0, 1.0]])

target.scale(0.15, center=target.get_center())
R = target.get_rotation_matrix_from_xyz((-np.pi/2,0,0))
target.rotate(R, center=(0,0,0))

print("Initial alignment")
evaluation = o3d.registration.evaluate_registration(source, target, threshold, trans_init)
print(evaluation)

#////////////////////////////////
print("Apply point-to-point ICP")
reg_p2p = o3d.registration.registration_icp(
        source, target, threshold, trans_init,
        o3d.registration.TransformationEstimationPointToPoint())
print(reg_p2p)
print("Transformation is:")
print(reg_p2p.transformation)
draw_registration_result(source, target, reg_p2p.transformation)
#////////////////////////////////


#////////////////////////////////
#reg_p2p = o3d.registration.registration_icp(source, target, threshold, trans_init,
#        o3d.registration.TransformationEstimationPointToPoint(),
#        o3d.registration.ICPConvergenceCriteria(max_iteration = 2000))
#print(reg_p2p)
#print("Transformation is:")
#print(reg_p2p.transformation)
#draw_registration_result(source, target, reg_p2p.transformation)
#////////////////////////////////

#////////////////////////////////
#print("Apply point-to-plane ICP")
#reg_p2l = o3d.registration.registration_icp(
#        source, target, threshold, trans_init,
#        o3d.registration.TransformationEstimationPointToPlane())
#print(reg_p2l)
#print("Transformation is:")
#print(reg_p2l.transformation)
#draw_registration_result(source, target, reg_p2l.transformation)