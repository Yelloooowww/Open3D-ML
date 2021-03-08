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
		# ROS Subscriber
		self.pub_markers = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=1)
		self.sub_points = rospy.Subscriber("/velodyne1/velodyne_points", PointCloud2, self.cb_points, queue_size=1)
		self.pub_points = rospy.Publisher('clustering_points', PointCloud2, queue_size=1)
		self.raw_point = []
		self.stamp = None
		self.header = None

		print("clustering init done")


	def cb_points(self,msg):
		# PointCloud2 to numpy
		cloud_points = []
		for p in point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
			if (p[0]>0) and (p[1]<2) and (p[1]>-2) and (p[2]<1) and (p[2]>-0.5):
				cloud_points.append(p)
		self.raw_point = np.array(cloud_points)
		self.header = msg.header

		# # downsample
		# pcd = o3d.geometry.PointCloud()
		# pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
		# downpcd = pcd.voxel_down_sample(voxel_size=0.1)
		# raw_point = np.asarray(downpcd.points)
		#
		# print(len(raw_point))


	def xyz_to_marker(self, x, y, z, id, color=[1, 0, 0],header=None):
			marker = Marker()
			if header: marker.header = header
			marker.type = marker.SPHERE
			marker.action = marker.ADD
			marker.pose.orientation.w = 1
			marker.pose.position.x = x
			marker.pose.position.y = y
			marker.pose.position.z = z
			marker.id = id
			marker.scale.x = 0.2
			marker.scale.y = 0.2
			marker.scale.z = 0.2
			marker.color.a = 1.0
			marker.color.r = color[0]
			marker.color.g = color[1]
			marker.color.b = color[2]
			return marker

	def xyzrgb_array_to_pointcloud2(self, points, colors, header=None):
		'''
		Create a sensor_msgs.PointCloud2 from an array
		of points.
		'''
		msg = PointCloud2()
		assert(points.shape == colors.shape)

		buf = []
		if header: msg.header = header
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


	def run_clustering(self):
		raw_point = self.raw_point
		header = self.header

		try:
			# numpy to open3d.PointCloud
			pcd = o3d.geometry.PointCloud()
			pcd.points = o3d.utility.Vector3dVector(raw_point)

			with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
				labels = np.array(pcd.cluster_dbscan(eps=0.25, min_points=15, print_progress=True))

			max_label = labels.max()
			print(f"point cloud has {max_label + 1} clusters")
			color_list = plt.get_cmap("tab20").colors

			if max_label + 1 > 0:
				group = [ [] for i in range(max_label + 1) ]
				pub_points = []
				pub_colors = []
				# find marker xyz
				for num,point in enumerate(pcd.points):
					(group[labels[num]]).append( np.asarray(point) )

				markerArray = MarkerArray()
				markerArray.markers = []
				id = 0
				for group_index in range(max_label+1):
					if len(group[group_index])>1500:

						xyz = np.mean(np.array(group[group_index]), axis=0)
						print('center',group_index+1,'at : ',xyz[0],xyz[1],xyz[2])
						id +=1

						color = list(color_list[group_index])
						markerArray.markers.append( self.xyz_to_marker(xyz[0],xyz[1],xyz[2],id=id,color=color,header=header) )

						pub_points = np.concatenate((pub_points, np.array(group[group_index]))) if not pub_points == [] else np.array(group[group_index])
						pub_colors = np.concatenate((pub_colors,np.array(np.array([color]*len(group[group_index]))))) if not pub_colors == [] else np.array(np.array([color]*len(group[group_index])))


				print(len(markerArray.markers))
				self.pub_markers.publish(markerArray)
				pub_msg = self.xyzrgb_array_to_pointcloud2( pub_points, pub_colors*255,header=header )
				self.pub_points.publish(pub_msg)
		except:
			rospy.logerr('Wrong')




if __name__ == "__main__":
	rospy.init_node("clustering")
	clu = Clustering()
	while not rospy.is_shutdown():
		clu.run_clustering()
	rospy.spin()
