import rosbag
import rospy
import rospkg

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/vicon/'
in_bag_path = path + 'test71.bag'
out_bag_path = path + 'test73.bag'
print('Modifying certain messages in a bag file')
print('Input bag: ' + in_bag_path)
print('Output bag: ' + out_bag_path)


with rosbag.Bag(in_bag_path, 'r') as inbag, rosbag.Bag(out_bag_path, 'w') as outbag:
    from tf2_msgs.msg import TFMessage
    count = [0, 0]
    for topic, msg, ts in inbag.read_messages():
        # Improve static tf
        if topic == '/tf':
            for i in range(len(msg.transforms)):
                if msg.transforms[i].header.frame_id == 'camera_mount_top' and msg.transforms[i].child_frame_id == 'camera_base':
                    msg.transforms[i].transform.translation.x = -0.028
                    msg.transforms[i].transform.translation.y = 0.023
                    msg.transforms[i].transform.rotation.z = 1.0 #0.99992744388 # 0.99992744388
                    msg.transforms[i].transform.rotation.w = 0.0 #0.01204603544 # 0.01204603544
                    count[0] += 1
                    if count[0] == 1:
                        print(msg.transforms[i])
        # Remove kalman_filter messages
        if '/kalman_filter' not in topic:
            outbag.write(topic, msg, ts)
        else:
            count[1] += 1
        
    
    
print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
    + str(count[1]) + ' messages have been deleted.')