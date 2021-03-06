#!/usr/bin/env python3
import numpy as np
import math
import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d
import open3d as o3d
import copy
import sys
import time
import struct
import ctypes
import numpy.lib.recfunctions as nlr
from matplotlib.cm import get_cmap


import numpy as np
import open3d as o3d

# Load binary point cloud
bin_pcd = np.fromfile("/home/yellow/dataset/kitti/data_odometry_velodyne/dataset/sequences/00/velodyne/000000.bin", dtype=np.float32)

# Reshape and drop reflection values
points = bin_pcd.reshape((-1, 4))[:, 0:3]

# Convert to Open3D point cloud
o3d_pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
#
# # Save to whatever format you like
# o3d.io.write_point_cloud("pointcloud.pcd", o3d_pcd)

o3d.visualization.draw_geometries([o3d_pcd])
