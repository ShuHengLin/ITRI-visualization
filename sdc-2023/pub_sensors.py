import os
import cv2
import rospy
import numpy as np
import pickle5 as pickle

from tqdm import tqdm
from lib.utils import *
import sensor_msgs.point_cloud2 as pcl2
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, Image, PointField
from cv_bridge import CvBridge

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

#==============================================================================================================================================

pointcloud_pub = rospy.Publisher('/pointcloud', PointCloud2, queue_size=10)
image_pub      = rospy.Publisher('/image',      Image,       queue_size=10)
radar_img_pub  = rospy.Publisher('/Radar_img',  Image,       queue_size=10)
rospy.init_node('talker', anonymous=True)
rate = rospy.Rate(1000)

img_id   = 1
lidar_id = 1

for radar_id in tqdm(range(500, len(radar_files))):

  if rospy.is_shutdown():
    break

  header = std_msgs.msg.Header()
  header.stamp = rospy.Time.now()
  header.frame_id = 'map'

  now_time = radar_timestamps[radar_id]

  # Read radar from file
  radar_img = cv2.imread(radar_root + radar_files[radar_id])
  radar_img_pub.publish(CvBridge().cv2_to_imgmsg(radar_img))


  # Read camera from file
  img_id = get_aligned_id(now_time, img_id, img_timestamps)
  img = cv2.imread(img_root + img_files[img_id])
  image_pub.publish(CvBridge().cv2_to_imgmsg(img))


  # Read LiDAR from file
  lidar_id = get_aligned_id(now_time, lidar_id, lidar_timestamps)
  with open(lidar_root + lidar_files[lidar_id], 'rb') as f:
    msg = pickle.load(f)
  pointcloud = read_point_from_msg(msg)
  pointcloud.transform(velodyne_to_base)
  fields = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)]
  pointcloud_pub.publish(pcl2.create_cloud(header, fields, pointcloud.points))

  rate.sleep()

###
# rosparam set /use_sim_time false
# python -B pub_lidar_camera.py
###
