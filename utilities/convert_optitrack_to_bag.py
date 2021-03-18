#!/usr/bin/python

import pandas as pd
import numpy as np
from numpy.linalg import norm
from scipy import signal
import os
import errno


import rospkg
import rospy
import rosbag
from tf.transformations import quaternion_from_matrix

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, \
                              Point, Quaternion, PointStamped, TransformStamped, Transform, \
                              Vector3, Vector3Stamped
from tf.msg import tfMessage
from std_msgs.msg import Header


trial_id_str = [str(i).rjust(3, '0') for i in range(226,227)]

df_header = ['', 't', 'KR1', 'KR1', 'KR1', 'KR2', 'KR2', 'KR2', 'KR3', 'KR3', 'KR3', 'KR4', 'KR4', 'KR4', 'KR5', 'KR5', 'KR5', 'LFF1', 'LFF1', 'LFF1', 'LFF2', 'LFF2', 'LFF2', 'LFF3', 'LFF3', 'LFF3', 'LHF1', 'LHF1', 'LHF1', 'LHF2', 'LHF2', 'LHF2', 'LHF3', 'LHF3', 'LHF3', 'Pelvis1', 'Pelvis1', 'Pelvis1', 'Pelvis2', 'Pelvis2', 'Pelvis2', 'Pelvis3', 'Pelvis3', 'Pelvis3', 'Pelvis4', 'Pelvis4', 'Pelvis4', 'RFF1', 'RFF1', 'RFF1', 'RFF2', 'RFF2', 'RFF2', 'RFF3', 'RFF3', 'RFF3', 'RHF1', 'RHF1', 'RHF1', 'RHF2', 'RHF2', 'RHF2', 'RHF3', 'RHF3', 'RHF3']

# Constants
global_bases = np.eye(3)
RotHomo = np.eye(4)

global_frame_id = 'optitrack'
topic_foot_pose_l = '/optitrack/foot_pose_l'
topic_foot_pose_r = '/optitrack/foot_pose_r'
topic_com = '/optitrack/com'
topic_ml_vec = '/optitrack/ml_vec'
robot_frame_id='/base_link' # '/base_link', '/camera_mount_top'
kinect_frame_id='/depth_camera_link_optitrack'

odom_topic = '/kinect_pose_estimator/odom'
inbag_relative_path = 'optitrack/odom'

# odom_topic = 'odom'
# inbag_relative_path = 'optitrack'


def createFootPoseList(FF, HF):
  # Find position
  alpha = 0.9
  pos = HF * alpha + FF * (1 - alpha)

  # Find orientation basis i
  vi = FF - HF
  vi /= norm(vi, axis=1)[:, np.newaxis]

  # Find orientation basis j
  vj = np.cross(vi, np.array([0., 0., -1.]))
  vj /= norm(vj, axis=1)[:, np.newaxis]

  # Find orientation basis k
  vk = np.cross(vi, vj)

  rows = pos.shape[0]
  res = []
  for i in range(0, rows):
    Rot = np.linalg.solve(global_bases, np.array([vi[i, ], vj[i, ], vk[i, ]])).T
    RotHomo[:3, :3] = Rot
    q = quaternion_from_matrix(RotHomo)
    res.append(Pose(position=Point(x=pos[i, 0], y=pos[i, 1], z=pos[i, 2]),
      orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
  return res

def createRobotTransformList(KR4, KR5, alpha_left=0.5, z_offset=0.):
  rows = KR4.shape[0]

  # Apply offset
  arr_offset = np.tile([[0., 0., z_offset]], (rows, 1))
  KR4 += arr_offset
  KR5 += arr_offset

  # Find position
  pos = KR4 * alpha_left + KR5 * (1 - alpha_left)

  # Find orientation basis j
  vj = KR4 - KR5
  vj[:, 2] = 0.
  vj /= norm(vj, axis=1)[:, np.newaxis]

  # Find orientation bases k and i
  vk = np.tile([[0., 0., 1.]], (rows, 1))
  vi = np.cross(vj, vk)

  res = []
  for i in range(0, rows):
    Rot = np.linalg.solve(global_bases, np.array([vi[i, ], vj[i, ], vk[i, ]])).T
    RotHomo[:3, :3] = Rot.T
    xyz = np.matmul(-Rot.T, pos[i, :])
    q = quaternion_from_matrix(RotHomo)
    res.append(Transform(translation=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
      rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
  return res

def createKinectTransformList(KR1, KR2, KR3, alpha_left=0.5, z_offset=0.):
  rows = KR1.shape[0]

  # Apply offset
  arr_offset = np.tile([[0., 0., z_offset]], (rows, 1))
  KR1 += arr_offset
  KR2 += arr_offset
  KR3 += arr_offset

  # Find position
  pos = KR1 * alpha_left + KR2 * (1 - alpha_left)

  # Find orientation basis j
  vi = KR1 - KR2
  vi[:, 2] = 0.
  vi /= norm(vi, axis=1)[:, np.newaxis]

  # Find orientation bases k and i
  vj = np.tile([[0., 0., -1.]], (rows, 1))
  vk = np.cross(vi, vj)

  res = []
  for i in range(0, rows):
    Rot = np.linalg.solve(global_bases, np.array([vi[i, ], vj[i, ], vk[i, ]])).T
    RotHomo[:3, :3] = Rot
    xyz = pos[i, :]
    q = quaternion_from_matrix(RotHomo)
    res.append(Transform(translation=Point(x=xyz[0], y=xyz[1], z=xyz[2]),
      rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
  return res

def find_time_offset(inbag, marker, ts_marker):
  period_optitrack = (ts_marker[-1] - ts_marker[0]) / (len(ts_marker) - 1)
  # Get vx from inbag odom and resample it per the frequency of ts_marker
  vx_odom = []
  ts_odom = []
  ts_odom_start = 15
  stamp0 = None
  for _, msg, _ in inbag.read_messages(topics=odom_topic):
    stamp = msg.header.stamp
    if not stamp0: stamp0 = stamp
    if (stamp - stamp0).to_sec() > ts_odom_start:
      vx_odom.append(msg.twist.twist.linear.x)
      ts_odom.append(stamp.to_sec())
  vx_odom = np.array(vx_odom)
  ts_odom = np.array(ts_odom)
  ts_odom_resampled = np.arange(ts_odom[0], ts_odom[-1], period_optitrack)
  vx_odom_resampled = np.interp(ts_odom_resampled, ts_odom, vx_odom)

  # Calculate vx from marker
  delta_marker = np.diff(marker, axis = 0)
  vx_marker = np.sqrt(delta_marker[:, 0]**2 + delta_marker[:, 1]**2) / period_optitrack

  # Make sure the lengths match
  n_points = min(len(ts_odom_resampled), len(ts_marker))
  if (n_points < len(ts_odom_resampled)):
    # ts_odom_resampled = ts_odom_resampled[0:n_points]
    vx_odom_resampled = vx_odom_resampled[0:n_points]
  else:
    # ts_marker = ts_marker[0:n_points]
    vx_marker = vx_marker[0:n_points]

  corr = signal.correlate(vx_odom_resampled, vx_marker, mode='same') \
    / np.sqrt(signal.correlate(vx_odom_resampled, vx_odom_resampled, mode='same')[int(n_points/2)] * 
              signal.correlate(vx_marker,         vx_marker,         mode='same')[int(n_points/2)])
  offset_arr = np.linspace(-0.5 * n_points * period_optitrack, 0.5 * n_points * period_optitrack, n_points)
  offset = offset_arr[np.argmax(corr)]
  return stamp0.to_sec() + ts_odom_start + offset, vx_marker

def convert(df, inbag, outbag):
  # Convert pandas.DataFrame to dictionary
  markers = {}
  for j in range(2, df.shape[1], 3):
    # [i', j', k'] = [i, -k, j]
    markers[df_header[j]] = df.iloc[:,[j, j+2, j+1]].values
    markers[df_header[j]][:, 1] *= -1.

  # Create some virtual markers
  LFF = (markers['LFF1'] + markers['LFF2'] + markers['LFF3']) / 3
  LHF = (markers['LHF1'] + markers['LHF2'] + markers['LHF3']) / 3
  RFF = (markers['RFF1'] + markers['RFF2'] + markers['RFF3']) / 3
  RHF = (markers['RHF1'] + markers['RHF2'] + markers['RHF3']) / 3
  Pelvis = (markers['Pelvis1'] + markers['Pelvis2'] + markers['Pelvis3'] + markers['Pelvis4']) / 4

  # Create unit vector point from right to left in mediolateral direction.
  ML_vec1 = markers['Pelvis1'] - markers['Pelvis2']
  ML_vec1[:, 2] = 0
  ML_vec1 /= norm(ML_vec1, axis=1)[:, np.newaxis]
  ML_vec2 = markers['Pelvis4'] - markers['Pelvis3']
  ML_vec2[:, 2] = 0
  ML_vec2 /= norm(ML_vec2, axis=1)[:, np.newaxis]
  ML_vec = (ML_vec1 + ML_vec2) / 2

  # 
  ts = df.iloc[:, 1].to_numpy()
  ts_offset, vx_robot = find_time_offset(inbag, markers['KR2'], ts)
  ts += ts_offset

  # Create message lists from markers
  LFPoseList = createFootPoseList(LFF, LHF)
  RFPoseList = createFootPoseList(RFF, RHF)
  RobotTransformList = createRobotTransformList(markers['KR4'], markers['KR5'], 0.5, -0.1)
  KinectTransformList = createKinectTransformList(markers['KR1'], markers['KR2'], markers['KR2'], 0.5, -0.01)

  # Write messages to outbag
  for i, (LFPose, RFPose, RobotTransform, KinectTransform) in enumerate(zip(LFPoseList, RFPoseList, RobotTransformList, KinectTransformList)):
    t = rospy.Time.from_sec(float(ts[i]))
    header = Header(stamp=t, frame_id='optitrack')
    foot_pose = PoseWithCovarianceStamped(header=header, pose=PoseWithCovariance(pose=LFPose))
    outbag.write(topic_foot_pose_l, foot_pose, t)
    foot_pose = PoseWithCovarianceStamped(header=header, pose=PoseWithCovariance(pose=RFPose))
    outbag.write(topic_foot_pose_r, foot_pose, t)
    com_point = PointStamped(header=header, point=Point(x=Pelvis[i, 0], y=Pelvis[i, 1], z=Pelvis[i, 2]))
    outbag.write(topic_com, com_point, t)
    ml_vector = Vector3Stamped(header=header, vector=Vector3(x=ML_vec[i, 0], y=ML_vec[i, 1], z=ML_vec[i, 2]))
    outbag.write(topic_ml_vec, ml_vector, t)
    tf_robot = tfMessage(transforms=[TransformStamped(
      header=Header(stamp=t, frame_id=robot_frame_id), 
      child_frame_id='optitrack', transform=RobotTransform)])
    outbag.write('/tf', tf_robot, t)
    tf_kinect = tfMessage(transforms=[TransformStamped(
      header=Header(stamp=t, frame_id='optitrack'), 
      child_frame_id=kinect_frame_id, transform=KinectTransform)])
    outbag.write('/tf', tf_kinect, t)
    if i < len(vx_robot):
      msg_vx_robot = Vector3Stamped(header=header, vector=Vector3(x=vx_robot[i]))
      outbag.write('/optitrack/vx_robot', msg_vx_robot, t)
    


def main():
  rp = rospkg.RosPack()
  path_gta = rp.get_path('gait_training_robot')
  for trial_id in trial_id_str:
    try:
      path_csv = os.path.join(path_gta, 'optitrack', 'csv', 'SCH_Trial_' + trial_id + '.csv')
      path_inbag = os.path.join(path_gta, 'bags', inbag_relative_path , 'data' + trial_id + '.bag')
      path_outbag = os.path.join(path_gta, 'optitrack', 'bags', 'data' + trial_id + '.bag')
      for path in [path_csv, path_inbag]:
        if not os.path.exists(path):
          raise IOError(errno.ENOENT, os.strerror(errno.ENOENT), path)
      with open(path_csv, 'r') as csvfile, rosbag.Bag(path_inbag, 'r') as inbag, rosbag.Bag(path_outbag, 'w') as outbag:
        df = pd.read_csv(csvfile, header=5)
        convert(df, inbag, outbag)
    except IOError as e:
      print(e)
      continue
    print('Processing trial {0:3s} complted'.format(trial_id))
    
      



if __name__ == '__main__':
  main()