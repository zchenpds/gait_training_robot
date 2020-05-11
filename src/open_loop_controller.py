#!/usr/bin/env python

import rospy
import rospkg
import math
import tf
import geometry_msgs.msg

def clamp(x, minx, maxx): 
    return max(minx, min(x, maxx))

def main():
    rospy.init_node('open_loop_controller')

    # Get parameters
    mode = rospy.get_param('mode', default='straight')
    terminal_distance = rospy.get_param('terminal_distance', default=3.0) # Desired distance
    v_max = rospy.get_param('v_max', default = 0.7)

    if mode != 'straight':
        rospy.logerr('Unknown mode %s' % mode)
        return
    
    # Create Transform Listener
    listener = tf.TransformListener()
    distance = 0.0

    # Create cmd_vel publisher
    pub_cmd_vel = rospy.Publisher('/Pioneer3AT/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    msg_cmd_vel = geometry_msgs.msg.Twist()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/base_link', '/odom', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Update the distance traveled
        if 'trans1' in locals():
            distance += math.hypot(trans[0] - trans1[0], trans[1] - trans1[1])
        else:
            distance = 0

        # Calculate the desired velocity
        e = max(terminal_distance - distance, 0.0)
        kp = 0.8
        u = clamp( kp * math.sqrt( 2.0 * 0.3 * e), 0.0, v_max)

        # Populate the message and publish
        msg_cmd_vel.linear.x = u
        msg_cmd_vel.linear.y = 0.0
        msg_cmd_vel.linear.z = 0.0
        msg_cmd_vel.angular.x = 0.0
        msg_cmd_vel.angular.y = 0.0
        msg_cmd_vel.angular.z = 0.0
        pub_cmd_vel.publish(msg_cmd_vel)

        # Terminate if error becomes smaller than tolerance
        if e < 0.2:
            break

        # Prepare for next iteration
        (trans1, rot1) = (trans, rot)
        rate.sleep()

    # Reset cmd_vel before exit
    msg_cmd_vel.linear.x = 0.0
    msg_cmd_vel.linear.y = 0.0
    msg_cmd_vel.linear.z = 0.0
    msg_cmd_vel.angular.x = 0.0
    msg_cmd_vel.angular.y = 0.0
    msg_cmd_vel.angular.z = 0.0
    pub_cmd_vel.publish(msg_cmd_vel)
    rospy.loginfo('cmd_vel set to (0.0, 0.0)')



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass