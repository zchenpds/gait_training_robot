import rosbag
import rospy
import rospkg

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/vicon/'
in_bag_path = path + 'test71.bag'
out_bag_path = path + 'test710.bag'
print('Modifying certain messages in a bag file')
print('Input bag: ' + in_bag_path)
print('Output bag: ' + out_bag_path)


with rosbag.Bag(in_bag_path, 'r') as inbag, rosbag.Bag(out_bag_path, 'w') as outbag:
    from tf2_msgs.msg import TFMessage
    count = [0, 0]
    for topic, msg, ts in inbag.read_messages():
        # Rename obsolete topics
        if '/sport_sole_kalman_filter' in topic:
            count[0] += 1
            topic.replace('/sport_sole_kalman_filter', '/kalman_filter')
        # Remove kalman_filter messages
        outbag.write(topic, msg, ts)
        
    
    
print('Migration completed: ' + str(count[0]) + ' messages have been modified. '
    + str(count[1]) + ' messages have been deleted.')