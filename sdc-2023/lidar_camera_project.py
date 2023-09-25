import os
import cv2
import matplotlib.pyplot as plt
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

gige3_to_sensorhead_1 = np.matrix([[ 0.00413371,  0.15402563,  0.98805821, -0.33334305],
                                   [ 0.99883745, -0.04809098,  0.00331796,  0.65132817],
                                   [ 0.04802774,  0.98689582, -0.15404536, -0.19554001],
                                   [ 0.,          0.,          0.,          1.        ]])

# /data_1TB_1/server_rosbag/sdc/vehicle-configuration/s3/video_capturing/lucid_cameras/gige_3_camera_info.yaml
calib = np.matrix([[1946.309450,    0.000000, 1017.834639, 0.000000],
                   [   0.000000, 1951.454069,  756.115010, 0.000000],
                   [   0.000000,    0.000000,    1.000000, 0.000000]])

#==============================================================================================================================================

fig = plt.figure(figsize=(20.48, 15.36), dpi=50)
ax = fig.add_subplot()

img_id   = 1
lidar_id = 1

for radar_id in tqdm(range(500, len(radar_files))):

  now_time = radar_timestamps[radar_id]

  # Read camera from file
  img_id = get_aligned_id(now_time, img_id, img_timestamps)
  img = cv2.imread(img_root + img_files[img_id])
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)


  # Read LiDAR from file
  lidar_id = get_aligned_id(now_time, lidar_id, lidar_timestamps)
  with open(lidar_root + lidar_files[lidar_id], 'rb') as f:
    msg = pickle.load(f)
  pointcloud = read_point_from_msg(msg)
  pointcloud.passthrough([0, 99, -99, 99, -99, 99])
  pointcloud.transform(velodyne_to_base)
  pointcloud.transform(get_inverse_tf(sensorhead_1_to_base))
  pointcloud.transform(get_inverse_tf(gige3_to_sensorhead_1))
  uv = project_to_image(pointcloud.points[:, 0:3], calib)


  # Output
  ax.clear()
  ax.imshow(img)
  ax.set_xlim(0, 2048)
  ax.set_ylim(1536, 0)
  ax.scatter([uv[:, 0]], [uv[:, 1]], c=pointcloud.points[:, 2], marker=',', s=10, edgecolors='none', alpha=0.7, cmap='jet')
  ax.set_axis_off()
#  fig.savefig('test.jpg', dpi=fig.dpi, bbox_inches='tight', pad_inches=0)
  plt.pause(0.01)
