#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d
import open3d as o3d
import copy
import sys
import time
import struct
import ctypes
import roslib
from geometry_msgs.msg import Transform, Vector3, Quaternion
import numpy.lib.recfunctions as nlr
from matplotlib.cm import get_cmap

class ply2pointcloud(object):
	def __init__(self):
		file_path = '/home/yellow/KPConv-PyTorch/Data/Stanford3dDataset_v1.2/input_0.020/Area_3.ply'
		print("Load a ply point cloud, print it, and render it")
		pcd = o3d.io.read_point_cloud(file_path)
		self.xyz_load = np.asarray(pcd.points)
		self.pub_msg = self.xyzrgb_array_to_pointcloud2( self.xyz_load, self.xyz_load)

		self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
		self.pub_points = rospy.Publisher('input_points', PointCloud2, queue_size=1)
		print("ply2pointcloud init done")

	def xyzrgb_array_to_pointcloud2(self,points, colors, stamp=None, frame_id='base_link', seq=None):
		'''
		Create a sensor_msgs.PointCloud2 from an array
		of points.
		'''
		msg = PointCloud2()
		assert(points.shape == colors.shape)

		buf = []

		if stamp:
			msg.header.stamp = stamp
		if frame_id:
			msg.header.frame_id = frame_id
		if seq:
			msg.header.seq = seq
		if len(points.shape) == 3:
			msg.height = points.shape[1]
			msg.width = points.shape[0]
		else:
			N = len(points)
			xyzrgb = np.array(np.hstack([points, colors]), dtype=np.float32)
			msg.height = 1
			msg.width = N

		msg.fields = [
			PointField('x', 0, PointField.FLOAT32, 1),
			PointField('y', 4, PointField.FLOAT32, 1),
			PointField('z', 8, PointField.FLOAT32, 1),
			PointField('r', 12, PointField.FLOAT32, 1),
			PointField('g', 16, PointField.FLOAT32, 1),
			PointField('b', 20, PointField.FLOAT32, 1)
		]
		msg.is_bigendian = False
		msg.point_step = 24
		msg.row_step = msg.point_step * N
		msg.is_dense = True;
		msg.data = xyzrgb.tostring()

		return msg



	def timer_callback(self,event):
		self.pub_points.publish(self.pub_msg)
		rospy.loginfo('pub_points')




if __name__ == "__main__":
	rospy.init_node("ply2pointcloud")
	Ply2PointCloud = ply2pointcloud()
	rospy.spin()
