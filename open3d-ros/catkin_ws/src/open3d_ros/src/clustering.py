#!/usr/bin/env python3
import rospy
import numpy as np
import math
from std_msgs.msg import Float32
from std_msgs.msg import Int32
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
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
import matplotlib.pyplot as plt


class Clustering(object):
	def __init__(self):

		self.pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=100)
		self.sub_points = rospy.Subscriber("input_points", PointCloud2, self.cb_points, queue_size=1)
		self.pub_points = rospy.Publisher('clustering_points', PointCloud2, queue_size=1)

		print("clustering init done")



	def cb_points(self,msg):
		assert isinstance(msg, PointCloud2)
		self.markerArray = MarkerArray()

		# PointCloud2 to numpy
		cloud_points = []
		for p in point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
			cloud_points.append(p)
		raw_point = np.array(cloud_points)

		# numpy to open3d.PointCloud
		pcd = o3d.geometry.PointCloud()
		pcd.points = o3d.utility.Vector3dVector(raw_point)

		with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
			labels = np.array(pcd.cluster_dbscan(eps=0.02, min_points=10, print_progress=True))

		max_label = labels.max()
		print(f"point cloud has {max_label + 1} clusters")

		colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
		colors[labels < 0] = 0
		pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

		pub_msg = self.xyzrgb_array_to_pointcloud2( np.array(pcd.points), np.array(pcd.colors),frame_id='base_link' )
		self.pub_points.publish(pub_msg)

		if max_label + 1 > 0:
			group = [ [] for i in range(max_label + 1) ]
			# find marker xyz
			for num,point in enumerate(pcd.points):
				(group[labels[num]]).append( np.asarray(point) )

			for group_index in range(max_label + 1):
				xyz = np.mean(np.array(group[group_index]), axis=0)
				print('center',group_index+1,'at : ',xyz[0],xyz[1],xyz[2])
				self.markerArray.markers.append( self.xyz_to_marker(xyz[0],xyz[1],xyz[2],id=group_index,frame_id='base_link') )

			self.pub_markers.publish(self.markerArray)
			# print(self.markerArray)

	def xyz_to_marker(self, x, y, z, id, color=[1, 0, 0], frame_id='base_link'):
			marker = Marker()
			marker.header.frame_id = frame_id
			marker.header.stamp = rospy.Time.now()
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.pose.orientation.w = 1
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = z
			marker.id = id
			marker.scale.x = 0.05
			marker.scale.y = 0.05
			marker.scale.z = 0.05
			marker.color.a = 1.0
			marker.color.r = color[0]
			marker.color.g = color[1]
			marker.color.b = color[2]
			return marker

	def xyzrgb_array_to_pointcloud2(self, points, colors, stamp=None, frame_id=None, seq=None):
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

if __name__ == "__main__":
	rospy.init_node("clustering")
	clustering = Clustering()
	rospy.spin()
