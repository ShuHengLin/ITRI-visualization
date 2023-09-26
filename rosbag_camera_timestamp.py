import os
import cv2
import rospy
import rosbag
import numpy as np

from cv_bridge import CvBridge
from tqdm import tqdm
from sensor_msgs.msg import Image, CompressedImage

bridge = CvBridge()
data_root = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/'

save_topic = "/lucid_cameras/gige_3/h265"
index = 0

timestamp_txt = 'camera_timestamp.txt'
if not os.path.isfile(timestamp_txt):
  os.mknod(timestamp_txt)

#==============================================================================================================================================

bag = rosbag.Bag(data_root + 'cameras.bag', "r")
bag_data = bag.read_messages(topics=[save_topic])

output = []
for topic, msg, t in tqdm(bag_data):

  if rospy.is_shutdown():
    break

  timestamp = str(t.secs) + "{:09d}".format(t.nsecs)[:4]
  if topic == save_topic:
    output.append(str(timestamp) + '\n')

open(timestamp_txt, "w").writelines(output)
