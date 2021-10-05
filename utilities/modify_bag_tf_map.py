#!/usr/bin/python2.7
import rosbag
import rospy
import rospkg

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/optitrack/'
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(194, 195)]]

def main():
    for bag_name in bag_names:
        try:
            modify_bag(bag_name, bag_name + 'x')
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
                for i in range(len(msg.transforms)):
                    if msg.transforms[i].header.frame_id == 'map' and msg.transforms[i].child_frame_id == 'odom':
                        count[0] += 1
                        if count[0] < 10:
                            continue
                        elif count[0] == 10:
                            tf_last = msg.transforms[i].transform
                        msg.transforms[i].transform.translation.x = tf_last.translation.x
                        msg.transforms[i].transform.translation.y = tf_last.translation.y
                        msg.transforms[i].transform.translation.z = 0.0
                        msg.transforms[i].transform.rotation.x = 0.0
                        msg.transforms[i].transform.rotation.y = 0.0
                        msg.transforms[i].transform.rotation.z = tf_last.rotation.z
                        msg.transforms[i].transform.rotation.w = tf_last.rotation.w
                        if count[0] == 10:
                            print(msg.transforms[i])
            outbag.write(topic, msg, ts)
            
        
        
    print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
        + str(count[1]) + ' messages have been deleted.')

if __name__ == '__main__':
    main()