#!/usr/bin/python

import pandas as pd
import numpy as np
from numpy.linalg import norm
import os

import rospkg
import rospy
import rosbag
from tf.transformations import *

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseWithCovariance, Pose, Point, Quaternion, PointStamped, TransformStamped, Transform
from tf.msg import tfMessage
from std_msgs.msg import Header


trial_id_str = [str(i).rjust(3, '0') for i in range(213,214)]

row3 = ['', 't', 'KR1', 'KR1', 'KR1', 'KR2', 'KR2', 'KR2', 'KR3', 'KR3', 'KR3', 'KR4', 'KR4', 'KR4', 'KR5', 'KR5', 'KR5', 'LFF1', 'LFF1', 'LFF1', 'LFF2', 'LFF2', 'LFF2', 'LFF3', 'LFF3', 'LFF3', 'LHF1', 'LHF1', 'LHF1', 'LHF2', 'LHF2', 'LHF2', 'LHF3', 'LHF3', 'LHF3', 'Pelvis1', 'Pelvis1', 'Pelvis1', 'Pelvis2', 'Pelvis2', 'Pelvis2', 'Pelvis3', 'Pelvis3', 'Pelvis3', 'Pelvis4', 'Pelvis4', 'Pelvis4', 'RFF1', 'RFF1', 'RFF1', 'RFF2', 'RFF2', 'RFF2', 'RFF3', 'RFF3', 'RFF3', 'RHF1', 'RHF1', 'RHF1', 'RHF2', 'RHF2', 'RHF2', 'RHF3', 'RHF3', 'RHF3']

t_ref = 3.0

# Constants
global_bases = np.eye(3)
RotHomo = np.eye(4)

global_frame_id = 'optitrack'
topic_foot_pose_l = '/optitrack/foot_pose_l'
topic_foot_pose_r = '/optitrack/foot_pose_r'
topic_com = '/optitrack/com'
robot_frame_id='/base_link'


def createFootPoseList(FF, HF):
  # Find position
  alpha = 0.9
  pos = HF * alpha + FF * (1 - alpha)

  #Find orientation basis i
  vi = FF - HF
  vi /= norm(vi, axis=1)[:, np.newaxis]

  #Find orientation basis j
  vj = np.cross(vi, np.array([0., 0., -1.]))
  vj /= norm(vj, axis=1)[:, np.newaxis]

  #Find orientation basis k
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

def createRobotTransformList(KR4, KR5):
  # Find position
  pos = (KR4 + KR5) / 2.

  #Find orientation basis j
  vj = KR4 - KR5
  vj[:, 2] = 0.
  vj /= norm(vj, axis=1)[:, np.newaxis]

  #Find orientation bases k and i
  vk = np.tile([[0., 0., 1.]], (len(vj), 1))
  vi = np.cross(vj, vk)

  rows = pos.shape[0]
  res = []
  for i in range(0, rows):
    Rot = np.linalg.solve(global_bases, np.array([vi[i, ], vj[i, ], vk[i, ]])).T
    RotHomo[:3, :3] = Rot
    q = quaternion_from_matrix(RotHomo)
    res.append(Transform(translation=Point(x=pos[i, 0], y=pos[i, 1], z=pos[i, 2]),
      rotation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])))
  return res


def convert(df, outbag):
  ts = df.iloc[:, 1].to_numpy() + t_ref
  # Convert pandas.DataFrame to dictionary
  markers = {}
  for j in range(2, df.shape[1], 3):
    # [i', j', k'] = [i, -k, j]
    markers[row3[j]] = df.iloc[:,[j, j+2, j+1]].values
    markers[row3[j]][:, 1] *= -1.
  LFF = (markers['LFF1'] + markers['LFF2'] + markers['LFF3']) / 3
  LHF = (markers['LHF1'] + markers['LHF2'] + markers['LHF3']) / 3
  RFF = (markers['RFF1'] + markers['RFF2'] + markers['RFF3']) / 3
  RHF = (markers['RHF1'] + markers['RHF2'] + markers['RHF3']) / 3
  Pelvis = (markers['Pelvis1'] + markers['Pelvis2'] + markers['Pelvis3'] + markers['Pelvis4']) / 4
  LFPoseList = createFootPoseList(LFF, LHF)
  RFPoseList = createFootPoseList(RFF, RHF)
  RobotTransformList = createRobotTransformList(markers['KR4'], markers['KR5'])
  for i, (LFPose, RFPose, RobotTransform) in enumerate(zip(LFPoseList, RFPoseList, RobotTransformList)):
    t = rospy.Time.from_sec(float(ts[i]))
    header = Header(stamp=t, frame_id='optitrack')
    foot_pose = PoseWithCovarianceStamped(header=header, pose=PoseWithCovariance(pose=LFPose))
    outbag.write(topic_foot_pose_l, foot_pose, t)
    foot_pose = PoseWithCovarianceStamped(header=header, pose=PoseWithCovariance(pose=RFPose))
    outbag.write(topic_foot_pose_r, foot_pose, t)
    com_point = PointStamped(header=header, point=Point(x=Pelvis[i, 0], y=Pelvis[i, 1], z=Pelvis[i, 2]))
    outbag.write(topic_com, com_point, t)
    tf_robot = tfMessage(transforms=[TransformStamped(header=header, child_frame_id=robot_frame_id, transform=RobotTransform)])
    outbag.write('/tf', tf_robot, t)
    


def main():
  rp = rospkg.RosPack()
  path_gta = rp.get_path('gait_training_robot')
  for trial_id in trial_id_str:
    path_csv = os.path.join(path_gta, 'optitrack', 'csv', 'SCH_Trial_' + trial_id + '.csv')
    path_bag = os.path.join(path_gta, 'optitrack', 'bags', 'data' + trial_id + '.bag')
    with open(path_csv, 'r') as csvfile, rosbag.Bag(path_bag, 'w') as outbag:
      df = pd.read_csv(csvfile, header=5)
      convert(df, outbag)
      
      



if __name__ == '__main__':
  main()