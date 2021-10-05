#!/usr/bin/env python

import rospy
import rospkg
import math
import tf
import geometry_msgs.msg
import sport_sole.msg
import visualization_msgs.msg

def clamp(x, minx, maxx): 
    return max(minx, min(x, maxx))

def main():
    rospy.init_node('simple_harmonic_controller')

    # Get parameters
    angular_amplitude = rospy.get_param('~angular_amplitude', default=math.pi/4)
    angular_frequency = rospy.get_param('~angular_frequency', default=math.pi/2)
    alpha_P = rospy.get_param('~alpha_P', default=2.0)
    terminal_time = rospy.get_param('~terminal_time', default=100.0)
    wait_for_required_msgs = rospy.get_param('~wait_for_required_msgs', default=True)

    # Define stage
    stage = 0
    stage_terminate = 10
    
    # Create Transform Listener
    listener = tf.TransformListener()

    # Create cmd_vel publisher
    pub_cmd_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    msg_cmd_vel = geometry_msgs.msg.Twist()

    # Wait for sport_sole msg
    if wait_for_required_msgs:
        rospy.loginfo('Waiting for body_tracking messages...')
        rospy.wait_for_message('/body_tracking_data', visualization_msgs.msg.MarkerArray)
        rospy.loginfo('body_tracking_data message received.')

    rate = rospy.Rate(10.0)
    
    # Keep track of elapsed time
    start_time = rospy.Time.now()

    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))#rospy.Time.now())# + rospy.Duration(0.1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        
        t = (rospy.Time.now() - start_time).to_sec()
        # Update state
        x = trans[0]
        y = trans[1]
        th = clamp(rot[2], -angular_amplitude, angular_amplitude)

        # Calculate the desired velocity
        if stage == 0:
            thd = angular_amplitude * math.sin(angular_frequency * t)
            wzd = angular_amplitude * angular_frequency * math.cos(angular_frequency * t)
            wz = wzd + alpha_P * (thd - th) 
            # rospy.loginfo('State: x = %f, y = %f, th = %f.' % (x, y, th) )
            rospy.loginfo('Error: thd - th = %f.' % (thd - th) )
            if t > terminal_time:
                stage += 1
        elif stage == 1:
            # Recover to initial state
            wz = alpha_P * (0 - th)
            if math.fabs(th) < 0.01:
                stage = stage_terminate
        else:
            #Time to shutdown
            pass

        # Populate the message and publish
        msg_cmd_vel.linear.x = 0.0
        msg_cmd_vel.angular.z = wz
        pub_cmd_vel.publish(msg_cmd_vel)
        # rospy.loginfo('cmd_vel set to (0.0, %f)' % wz)

        # Terminate if error becomes smaller than tolerance
        if stage == stage_terminate:
            break

        # Prepare for next iteration
        rate.sleep()

    # Reset cmd_vel before exit
    msg_cmd_vel.linear.x = 0.0
    pub_cmd_vel.publish(msg_cmd_vel)
    rospy.loginfo('cmd_vel set to (0.0, 0.0)')



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass