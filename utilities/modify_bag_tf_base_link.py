#!/usr/bin/python2.7
# Delete all tfs from 'odom' to 'base_link'

import rosbag
import rospy
import rospkg
import tf2_msgs.msg

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/optitrack/'
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(222, 230)]]
# bag_names = ['test' + s for s in [str(i).rjust(3, '0') for i in range(10, 11)]]

# bag_names = ['data_shoe01', 'data_shoe02', 'data_shoe03']
def main():
    for bag_name in bag_names:
        try:
            modify_bag(bag_name, bag_name + 'b')
        except:
            print('Cannot open bag: ' + bag_name)

def modify_bag(in_bag_name, out_bag_name):
    in_bag_path = path + in_bag_name + '.bag'
    out_bag_path = path + out_bag_name + '.bag'
    print('Modifying certain messages in a bag file')
    print('Input bag:  ' + in_bag_path)
    print('Output bag: ' + out_bag_path)


    with rosbag.Bag(in_bag_path, 'r') as inbag, rosbag.Bag(out_bag_path, 'w') as outbag:
        # from tf2_msgs.msg import TFMessage
        count = [0, 0]
        for topic, msg, ts in inbag.read_messages():
            # Improve static tf
            if topic == '/tf':
                msg_new = tf2_msgs.msg.TFMessage()
                for i in range(len(msg.transforms)):
                    if not (msg.transforms[i].header.frame_id == 'odom' and msg.transforms[i].child_frame_id == 'base_link'):
                        msg_new.transforms.append(msg.transforms[i])
                    else:
                        count[1] += 1
                        # if count[1] == 1:
                        #     print(msg.transforms[i])
                msg = msg_new
            # elif topic == "/odom":
            #     msg.header.stamp += rospy.Duration.from_sec(odom_lead_time_in_secs)
            if topic == '/tf' and not msg.transforms:
                continue
            if topic == '/sport_sole_publisher/sport_sole':
                msg.header.frame_id = 'camera_base'
                count[0] += 1
            outbag.write(topic, msg, ts)
            
        
        
    print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
        + str(count[1]) + ' messages have been deleted.')

if __name__ == '__main__':
    main()