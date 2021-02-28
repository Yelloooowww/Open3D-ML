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

def xyzrgb_array_to_pointcloud2(points, colors, stamp=None, frame_id=None, seq=None):
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




class KPFCNN_d435(object):
	def __init__(self):
		# for model
		cfg_file = "/home/yellow/Open3D-ML/ml3d/configs/kpconv_s3dis.yml"
		cfg = _ml3d.utils.Config.load_from_file(cfg_file)
		self.model = ml3d.models.KPFCNN(**cfg.model)
		self.pipeline = ml3d.pipelines.SemanticSegmentation(self.model, device="gpu")

		# download the weights.
		print('# download the weights.')
		ckpt_folder = "./logs/"
		os.makedirs(ckpt_folder, exist_ok=True)
		ckpt_path = ckpt_folder + "kpconv_s3dis_202010091238.pth"
		url = "https://storage.googleapis.com/open3d-releases/model-zoo/kpconv_s3dis_202010091238.pth"
		if not os.path.exists(ckpt_path):
			cmd = "wget {} -O {}".format(url, ckpt_path)
			os.system(cmd)
		# load the parameters.
		print('# load the parameters.')
		self.pipeline.load_ckpt(ckpt_path=ckpt_path)

		# ROS Subscriber
		self.sub_points = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.cb_points, queue_size=1)
		self.pub_points = rospy.Publisher('predict_points', PointCloud2, queue_size=1)
		print("KPFCNN_d435 init done")



	def cb_points(self,msg):
		assert isinstance(msg, PointCloud2)

		# PointCloud2 to numpy
		cloud_points = []
		for p in point_cloud2.read_points(msg, field_names = ("x", "y", "z"), skip_nans=True):
			cloud_points.append(p)
		raw_point = np.array(cloud_points)

		# downsample
		# pcd = o3d.geometry.PointCloud()
		# pcd.points = o3d.utility.Vector3dVector(np.array(cloud_points))
		# downpcd = pcd.voxel_down_sample(voxel_size=0.05)
		# raw_point = np.asarray(downpcd.points)
		
		print(len(raw_point))

		# prepare input data
		feat = np.zeros((len(raw_point),3))
		label = np.zeros(len(raw_point))
		data = {'point':raw_point , 'feat':feat ,'label':label}

		#run
		results = self.pipeline.run_inference(data)

		# visualize result with rviz
		r_array = results['predict_labels']
		rs_array = results['predict_scores']
		goal_points_color_list = raw_point.copy()

		goal_class = 11
		for num,points in enumerate(goal_points_color_list):
			# if not (r_array[num] == goal_class and rs_array[num][goal_class]>0.7):
			if not (r_array[num] == goal_class):
				goal_points_color_list[num] = [0,0,0]
			else :
				goal_points_color_list[num] = [1,0,0]

		pub_msg = xyzrgb_array_to_pointcloud2( raw_point, goal_points_color_list,frame_id='camera_color_optical_frame' )
		self.pub_points.publish(pub_msg)




if __name__ == "__main__":
	rospy.init_node("kpfcnn")
	kpfcnn = KPFCNN_d435()
	rospy.spin()
