import os
import cv2

import numpy as np
import pickle5 as pickle

from tqdm import tqdm
from lib.utils import *
import sensor_msgs.point_cloud2 as pcl2

#============================================================================================================================================== 

img_root   = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/single_sensor_data/bag1/camera/rosbag/'
lidar_root = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/single_sensor_data/bag1/velodyne/'
radar_root = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/single_sensor_data/bag1/radar_cart_0_n200/'

img_files   = sorted(os.listdir(img_root))
lidar_files = sorted(os.listdir(lidar_root))
radar_files = sorted(os.listdir(radar_root))

img_timestamps   = create_timestamps(img_files,   'image')
lidar_timestamps = create_timestamps(lidar_files, 'lidar')
radar_timestamps = create_timestamps(radar_files, 'radar')

velodyne_to_base     = np.matrix([[ 9.99508523e-01,  3.13481023e-02,  9.95593843e-05,  1.16372713e+00],
                                  [-3.13465186e-02,  9.99481426e-01, -7.36717432e-03,  1.04705414e-02],
                                  [-3.30454690e-04,  7.36043268e-03,  9.99972857e-01,  1.68658197e+00],
                                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

sensorhead_1_to_base = np.matrix([[ 9.85584767e-01, -2.80266262e-17, -1.69182349e-01,  1.20699729e+00],
                                  [ 2.69448762e-04, -9.99998732e-01,  1.56969445e-03, -2.25553480e-03],
                                  [-1.69182134e-01, -1.59265292e-03, -9.85583517e-01,  1.41621060e+00],
                                  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

sensorhead_2_to_base = np.matrix([[-0.98696216, -0.00420745,  0.16089745, -0.38999448],
                                  [-0.00370131,  0.99998722,  0.00344531,  0.01492972],
                                  [-0.16090989,  0.00280486, -0.98696512,  1.40854092],
                                  [ 0.,          0.,          0.,          1.        ]])

from scipy.spatial.transform import Rotation
Rot = np.matrix(np.zeros((4, 4)))
Rot[:3, :3] = Rotation.from_euler('xzy', [180, 0, 180], degrees=True).as_matrix()
Rot[3, 3] = 1

#==============================================================================================================================================

img_id   = 1
lidar_id = 1

for radar_id in tqdm(range(500, len(radar_files))):

  now_time = radar_timestamps[radar_id]

  # Read radar from file
  radar_img = cv2.imread(radar_root + radar_files[radar_id])


  # Read camera from file
  img_id = get_aligned_id(now_time, img_id, img_timestamps)
  img = cv2.imread(img_root + img_files[img_id])


  # Read LiDAR from file
  lidar_id = get_aligned_id(now_time, lidar_id, lidar_timestamps)
  with open(lidar_root + lidar_files[lidar_id], 'rb') as f:
    msg = pickle.load(f)
  pointcloud = read_point_from_msg(msg)
  pointcloud.transform(velodyne_to_base)
  pointcloud.transform(Rot)

  # Voxelize and create LiDAR image
  voxel = pointcloud.voxelize((0.175, 0.175, 0.175),
                              extents=np.array([[-199.7,  0.0],
                                                [- 99.9, 99.9],
                                                [  -2.0,  5.0],]
                                              ),
                              return_indices=False,
                              )
  lidar_img = np.zeros((1142, 1142, 3)).astype(np.uint8)
  lidar_img[:, :, 2] = np.sum(voxel, axis=2) * 255


  # Output
  output = cv2.addWeighted(lidar_img, 0.5, radar_img, 1.0, 0)
  cv2.namedWindow ('camera', 0)
  cv2.resizeWindow('camera', [img.shape[1] // 2, img.shape[0] // 2])
  cv2.moveWindow  ('camera', 1200, 0)

  cv2.namedWindow ('output', 0)
  cv2.resizeWindow('output', [900, 900])
  cv2.moveWindow  ('output', 0, 0)
  
  cv2.imshow('output', output)
  cv2.imshow('camera', img)
  cv2.waitKey(1)
