import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d

import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
import copy
import os
import sys

cfg_file = "/home/yellow/Open3D-ML/ml3d/configs/kpconv_s3dis.yml"
cfg = _ml3d.utils.Config.load_from_file(cfg_file)

model = ml3d.models.KPFCNN(**cfg.model)
cfg.dataset['dataset_path'] = "/home/yellow/KPConv-PyTorch/Data/Stanford3dDataset_v1.2/"
dataset = ml3d.datasets.S3DIS(cfg.dataset.pop('dataset_path', None), **cfg.dataset)
pipeline = ml3d.pipelines.SemanticSegmentation(model, dataset=dataset, device="gpu", **cfg.pipeline)

# download the weights.
print('# download the weights.')
ckpt_folder = "./logs/"
os.makedirs(ckpt_folder, exist_ok=True)
ckpt_path = ckpt_folder + "kpconv_s3dis_202010091238.pth"
pointpillar_url = "https://storage.googleapis.com/open3d-releases/model-zoo/kpconv_s3dis_202010091238.pth"
if not os.path.exists(ckpt_path):
    cmd = "wget {} -O {}".format(pointpillar_url, ckpt_path)
    os.system(cmd)

# load the parameters.
print('# load the parameters.')
pipeline.load_ckpt(ckpt_path=ckpt_path)

# test_split = dataset.get_split("test")
# data = test_split.get_data(0)
# run inference on a single example.
# returns dict with 'predict_labels' and 'predict_scores'.
# print('# run inference on a single example.')
# result = pipeline.run_inference(data)
#
# # evaluate performance on the test set; this will write logs to './logs'.
# print('# evaluate performance on the test set; this will write logs to ./logs.')
# pipeline.run_test()
# print('Finish')
print("Load a ply point cloud, print it, and render it")
# pcd = o3d.io.read_point_cloud("/home/yellow/Open3D-ML/data/demo/fragment.ply")
pcd = o3d.io.read_point_cloud("/home/yellow/KPConv-PyTorch/Data/Stanford3dDataset_v1.2/input_0.030/Area_5.ply")
downpcd = pcd.voxel_down_sample(voxel_size=0.05)
# o3d.visualization.draw_geometries([pcd],
#                                   zoom=0.3412,
#                                   front=[0.4257, -0.2125, -0.8795],
#                                   lookat=[2.6172, 2.0475, 1.532],
#                                   up=[-0.0694, -0.9768, 0.2024])
p_color = np.asarray(downpcd.colors)
p = np.asarray(downpcd.points)

f = np.zeros((len(p),3))
l = np.zeros(len(p))
print('print(len(p))=',len(p))
d = {'point':p , 'feat':f ,'label':l}
results = pipeline.run_inference(d)
# print(results)
# print(results['predict_scores'])

r_array = results['predict_labels']
rs_array = results['predict_scores']
r_pcd = downpcd

color_list = [[1,0,0],[0,1,0],[0,0,1],[1,1,0],[1,0,1],[0,1,1],[1,1,1],[0.5,0.5,0],[0.5,0,0.5],[0,0.5,0.5],[0.5,0,0],[0,0.5,0],[0,0,0.5],[0.5,0.5,0.5],[0.25,0.25,0],[0.25,0,0.25],[0,0.25,0.25],[0.25,0.25,0.25],[0.25,0,0],[0,0.25,0]]

for num,points in enumerate(p_color):
    r_pcd.colors[num] = color_list[ r_array[num] ]

    # if r_array[num] == 12:
    #     # print('red:',rs_array[num])
    #     r_pcd.colors[num] = [1,0,0]
    # else :
    #     r_pcd.colors[num] = [0,0,0]

# print(np.asarray(r_pcd.colors))
o3d.visualization.draw_geometries([r_pcd])
o3d.io.write_point_cloud("pridict_result.pcd", r_pcd)
print('write_point_cloud')
