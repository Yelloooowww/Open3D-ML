# import open3d.ml as _ml3d
# import open3d.ml.torch as ml3d # or open3d.ml.tf as ml3d
# import os
#
#
# framework = "torch" # or tf
# cfg_file = "ml3d/configs/kpconv_s3dis.yml"
# cfg = _ml3d.utils.Config.load_from_file(cfg_file)
#
# # fetch the classes by the name
# Pipeline = _ml3d.utils.get_module("pipeline", cfg.pipeline.name, framework)
# Model = _ml3d.utils.get_module("model", cfg.model.name, framework)
# Dataset = _ml3d.utils.get_module("dataset", cfg.dataset.name)
#
# # use the arguments in the config file to construct the instances
# cfg.dataset['dataset_path'] = "/home/yellow/KPConv-PyTorch/Data/Stanford3dDataset_v1.2"
# dataset = Dataset(cfg.dataset.pop('dataset_path', None), **cfg.dataset)
# model = Model(**cfg.model)
# pipeline = Pipeline(model, dataset, **cfg.pipeline)
#
# # download the weights.
# ckpt_folder = "./logs/"
# os.makedirs(ckpt_folder, exist_ok=True)
# ckpt_path = ckpt_folder + "randlanet_semantickitti_202009090354utc.pth"
# randlanet_url = "https://storage.googleapis.com/open3d-releases/model-zoo/randlanet_semantickitti_202009090354utc.pth"
# if not os.path.exists(ckpt_path):
#     cmd = "wget {} -O {}".format(randlanet_url, ckpt_path)
#     os.system(cmd)
#
# # ckpt_path = "/home/yellow/Open3D-ML/logs/KPFCNN_S3DIS_torch/checkpoint/chkp_0500.tar"
# # # load the parameters.
# # pipeline.load_ckpt(ckpt_path=ckpt_path)
# print('prepare data')
# test_split = dataset.get_split("test")
# print('split')
# data = test_split.get_data(0)
# print('data ok')
#
# # run inference on a single example.
# # returns dict with 'predict_labels' and 'predict_scores'.
# result = pipeline.run_inference(data)
# print('run test')
# # evaluate performance on the test set; this will write logs to './logs'.
# pipeline.run_test()
# print('finish')

import os
import open3d.ml as _ml3d
import open3d.ml.torch as ml3d

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

test_split = dataset.get_split("test")
data = test_split.get_data(0)

# run inference on a single example.
# returns dict with 'predict_labels' and 'predict_scores'.
print('# run inference on a single example.')
result = pipeline.run_inference(data)

# evaluate performance on the test set; this will write logs to './logs'.
print('# evaluate performance on the test set; this will write logs to ./logs.')
pipeline.run_test()
print('Finish')
