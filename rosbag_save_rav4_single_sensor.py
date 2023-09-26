import os
import cv2
import rospy
import rosbag
import numpy as np

from cv_bridge import CvBridge
from tqdm import tqdm
from sensor_msgs.msg import Image, CompressedImage, PointCloud2

import genpy
import pickle
import importlib

def obj_from_str(module_name, class_name, *args, **kwargs):
    # Get an instance of module_name.class_name
    mod = importlib.import_module(module_name)
    obj = getattr(mod, class_name)(*args, **kwargs)
    return obj


def rewrite(x):
    # if isinstance(x, (bool, int, long, float, complex, str, genpy.Time, genpy.Duration, rospy.Time, rospy.Duration)):
    if isinstance(x, (bytes, bool, int, float, complex, str, genpy.Time, genpy.Duration, rospy.Time, rospy.Duration)):
        # A primitive type (see http://wiki.ros.org/msg)
        return x
    elif isinstance(x, list):
        return [rewrite(item) for item in x]
    elif isinstance(x, tuple):
        return tuple(rewrite(item) for item in x)
    elif hasattr(x, '_type') and hasattr(x, '__slots__'):
        # A ROS message type
        module_name, class_name = x._type.split('/')
        y = obj_from_str(module_name + '.msg', class_name)

        assert x.__slots__ == y.__slots__

        # Recursively rewrite fields
        for slot in x.__slots__:
            setattr(y, slot, rewrite(getattr(x, slot)))

        return y
    else:
        raise NotImplementedError("Type '{}' not handled".format(type(x)))
        
#==============================================================================================================================================

data_root = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/'

output_root = data_root + 'single_sensor_data/bag1'
if not os.path.exists(output_root):
  os.mkdir(output_root)
  
output_root_radar = output_root + '/radar'
if not os.path.exists(output_root_radar):
  os.mkdir(output_root_radar)

output_root_baraja_1 = output_root + '/baraja_1'
if not os.path.exists(output_root_baraja_1):
  os.mkdir(output_root_baraja_1)
  
output_root_baraja_2 = output_root + '/baraja_2'
if not os.path.exists(output_root_baraja_2):
  os.mkdir(output_root_baraja_2)
  
output_root_velodyne = output_root + '/velodyne'
if not os.path.exists(output_root_velodyne):
  os.mkdir(output_root_velodyne)

#==============================================================================================================================================

# for saving radar image and msg(radar, lidar, camera) pkl file          
def bag_record(bag_file):

  bag = rosbag.Bag(bag_file, "r")
  bag_data = bag.read_messages(topics=['/Navtech/Polar', '/velodyne_points', '/baraja_lidar/sensorhead_1', '/baraja_lidar/sensorhead_2'])

  for topic, msg, t in tqdm(bag_data):
    
    if rospy.is_shutdown():
        break

    timestamp = str(t.secs) + "{:09d}".format(t.nsecs)[:4]

    if topic == "/Navtech/Polar":
      msg = rewrite(msg)
      with open(output_root_radar + '/' + timestamp + '.pkl', 'wb') as f:
        pickle.dump(msg, f, pickle.HIGHEST_PROTOCOL)

    if topic == "/baraja_lidar/sensorhead_1":
      msg = rewrite(msg)
      with open(output_root_baraja_1  + '/' + timestamp + '.pkl', 'wb') as f:
        pickle.dump(msg, f, pickle.HIGHEST_PROTOCOL)

    if topic == "/baraja_lidar/sensorhead_2":
      msg = rewrite(msg)
      with open(output_root_baraja_2  + '/' + timestamp + '.pkl', 'wb') as f:
        pickle.dump(msg, f, pickle.HIGHEST_PROTOCOL)

    if topic == "/velodyne_points":
      msg = rewrite(msg)
      with open(output_root_velodyne  + '/' + timestamp + '.pkl', 'wb') as f:
        pickle.dump(msg, f, pickle.HIGHEST_PROTOCOL)

#==============================================================================================================================================

if __name__ == '__main__':
  bag_file = data_root + 'lidars.bag'
  bag_record(bag_file)

