import numpy as np
import sensor_msgs.point_cloud2 as pcl2

from .Point_utils import Pointcloud


def create_timestamps(files, source):

  timestamps = []

  for file in files:
    if   source == 'image' and file.endswith('.jpg'):
        name = file.split('.jpg')[0]

    elif source == 'lidar' and file.endswith('.pkl'):
        name = file.split('.pkl')[0]

    elif source == 'radar' and file.endswith('.png'):
        name = file.split('.png')[0]

    timestamp = name[:10] + '.' + name[10:]
    timestamps.append(float(timestamp))

  return timestamps


def get_aligned_id(now_time, now_idx, timestamps_list):

  for i in range(now_idx, len(timestamps_list)):
    if timestamps_list[i - 1] < now_time and timestamps_list[i] > now_time:
      now_idx = i - 1
      break

  return now_idx


def read_point_from_msg(msg):

  points_list = []
  for point in pcl2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "intensity")):

    if point[0] == 0 and point[1] == 0 and point[2] == 0:
      continue

    if np.abs(point[0]) < 2.0 and np.abs(point[1]) < 1.5:
      continue

    points_list.append(point)

  return Pointcloud(np.asarray(points_list, dtype=np.float32))


def get_inverse_tf(T):
  """Returns the inverse of a given 4x4 homogeneous transform.
  Args:
      T (np.ndarray): 4x4 transformation matrix
  Returns:
      np.ndarray: inv(T)
  """
  T2 = T.copy()
  T2[:3, :3] = T2[:3, :3].transpose()
  T2[:3, 3:] = -1 * T2[:3, :3] @ T2[:3, 3:]

  return T2


def project_to_image(points, P):

  pts_3d = np.copy(points)
  pts_3d *= 1 / pts_3d[:, 2:3]
  pts_3d_homo = np.concatenate([pts_3d, np.ones((pts_3d.shape[0], 1))], axis=1)
  pts_2d = np.dot(P, pts_3d_homo.transpose(1, 0)).transpose(1, 0)
  pts_2d = pts_2d[:, :2] / pts_2d[:, 2:]

  return pts_2d

