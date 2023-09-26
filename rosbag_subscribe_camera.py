import os
import cv2
import rospy

import numpy as np

from cv_bridge import CvBridge
from tqdm import tqdm
from sensor_msgs.msg import Image, CompressedImage

bridge = CvBridge()
data_root = '/data_1TB_1/server_rosbag/DCV/7685ed5a9daf9504e1cda45a8ffa73402b94104af333f44ccb97f07da6a5cd9b/'

output_root = data_root + 'single_sensor_data/bag1'
if not os.path.exists(output_root):
  os.mkdir(output_root)

output_root_camera = output_root + '/camera'
if not os.path.exists(output_root_camera):
  os.mkdir(output_root_camera)

output_root_camera_rosbag = output_root_camera + '/rosbag'
if not os.path.exists(output_root_camera_rosbag):
  os.mkdir(output_root_camera_rosbag)

timestamp_txt = 'camera_timestamp.txt'
read = [] 
for line in open(timestamp_txt, "r"):
  read.append(line.replace("\n", ""))

index = 0

#==============================================================================================================================================

def callback(data):

  global index
  img = np.frombuffer(data.data, np.uint8).reshape(data.height, data.width, -1)
  img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
  """
  cv2.namedWindow("window",0)
  cv2.resizeWindow("window", 1800, 800)
  cv2.imshow("window", img)
  cv2.waitKey(1)
  """
  cv2.imwrite(output_root_camera_rosbag  + '/' + str(read[index]) + '.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])
  print(str(read[index]))
  index += 1


if __name__ == '__main__':
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/lucid_cameras/gige_3/h265/rgb8", Image, callback, queue_size=None)
  rospy.spin()

