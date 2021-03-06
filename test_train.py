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


# print( 'use a cache for storing the results of the preprocessing (default path is ./logs/cache)' )
# dataset = ml3d.datasets.S3DIS(dataset_path='/home/yellow/dataset/Stanford3dDataset_v1.2', use_cache=True)
#
# print( 'create the model with random initialization.')
# model = ml3d.models.KPFCNN()
#
# print('pipeline = ml3d.pipelines.SemanticSegmentation(model=model, dataset=dataset, max_epoch=100)')
# pipeline = ml3d.pipelines.SemanticSegmentation(model=model, dataset=dataset, max_epoch=100)
#
# print('prints training progress in the console.')
# pipeline.run_train()
# print('Done')
# use a cache for storing the results of the preprocessing (default path is './logs/cache')
dataset = ml3d.datasets.SemanticKITTI(dataset_path='/path/to/SemanticKITTI/', use_cache=True)

# create the model with random initialization.
model = RandLANet()

pipeline = SemanticSegmentation(model=model, dataset=dataset, max_epoch=100)

# prints training progress in the console.
pipeline.run_train()
