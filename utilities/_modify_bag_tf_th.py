#!/usr/bin/env python
import rosbag
import rospy
import rospkg
import numpy as np
import math

# Complementary filter params
mu = 0.01

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/new/'
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(38, 61)]]

def main():
    for bag_name in bag_names:
        modify_bag(bag_name, bag_name + 'm'')

def contains_transform(msg, parent_frame_id, child_frame_id):
    for i in range(len(msg.transforms)):
        if msg.transforms[i].header.frame_id == 'odom' and msg.transforms[i].child_frame_id == 'base_link':
            # print(i, len(msg.transforms))
            return True
    return False

def set_transform_th(msg, parent_frame_id, child_frame_id, th):
    flag = False
    for i in range(len(msg.transforms)):
        if msg.transforms[i].header.frame_id == 'odom' and msg.transforms[i].child_frame_id == 'base_link':
            flag = True
            # Measurement
            th_m = math.atan2(msg.transforms[i].transform.rotation.z, msg.transforms[i].transform.rotation.w) * 2
            # Correction
            th = th * (1 - mu) + th_m * mu
            # Debug output
            # print(th_m, th)
            # Assign to ret
            msg.transforms[i].transform.rotation.z = math.sin(th / 2.0)
            msg.transforms[i].transform.rotation.w = math.cos(th / 2.0)
    assert(flag)
    return (msg, th)

def modify_bag(in_bag_name, out_bag_name):
    in_bag_path = path + in_bag_name + '.bag'
    out_bag_path = path + out_bag_name + '.bag'
    print('Modifying certain messages in a bag file')
    print('Input bag:  ' + in_bag_path)
    print('Output bag: ' + out_bag_path)


    with rosbag.Bag(in_bag_path, 'r') as inbag, rosbag.Bag(out_bag_path, 'w') as outbag:
        # Prepare for calculatign gyroscope offset
        wz0_sum = 0.0
        wz0_count = 0
        wz0_count_max = 3000
        wz0 = 0.0
        # Prepare for filtering the yaw angles
        th = 0.0
        dt = 6.066e-4
        msg_tuple = []
        write_ready = True
        kimu_ts = rospy.Time(0)
        odom_ts = rospy.Time(0)
        # from tf2_msgs.msg import TFMessage
        count = [0, 0]
        kimu_counter = 0
        tf_msg = None
        for topic, msg, ts in inbag.read_messages():
            # Improve static tf
            if topic == '/tf':
                if contains_transform(msg, 'odom', 'base_link') and not msg_tuple:
                    odom_ts = ts
                    if odom_ts > kimu_ts:
                        write_ready = False
            elif topic == '/imu':
                kimu_ts = ts
                if wz0_count < wz0_count_max:
                    wz0_sum += msg.angular_velocity.z
                    if wz0_count == 0:
                        kimu_ts0 = ts
                    wz0_count += 1
                elif wz0_count == wz0_count_max:
                    dt = (ts - kimu_ts0).to_sec() / wz0_count
                    wz0 = wz0_sum / wz0_count
                    wz0_count += 1
                else:
                    # Input
                    wz = - (msg.angular_velocity.z - wz0)
                    # Predict a priori estimate
                    th += wz * dt
                    kimu_counter += 1
                    # if kimu_counter % 20 == 1:
                    #     tf_topic = "/tf"
                    #     if tf_msg is not None:
                    #         tf_msg = set_transform_th(tf_msg, 'odom', 'base_link', th)
                    #         msg_tuple.append((tf_topic, tf_msg, ts))

                    if kimu_ts > odom_ts:
                        if msg_tuple:
                            tf_msg, th = set_transform_th(msg_tuple[0][1], 'odom', 'base_link', th)
                            msg_tuple[0] = (msg_tuple[0][0], tf_msg, msg_tuple[0][2])
                            write_ready = True
                            count[0] += 1
        
            # Buffer messages to be written
            msg_tuple.append((topic, msg, ts))
            if write_ready == True:
                for topic, msg, ts in msg_tuple:
                    outbag.write(topic, msg, ts)
                msg_tuple = []
            
        
        
    print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
        + str(count[1]) + ' messages have been deleted.')

if __name__ == '__main__':
    main()