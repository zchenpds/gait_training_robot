#!/usr/bin/env python

import rospy
import rospkg
import math
import tf
import geometry_msgs.msg
import sport_sole.msg

def clamp(x, minx, maxx): 
    return max(minx, min(x, maxx))

def main():
    rospy.init_node('open_loop_controller')

    # Get parameters
    mode = rospy.get_param('~mode', default='straight')
    terminal_distance = rospy.get_param('~terminal_distance', default=3.0) # Desired distance
    v_max = rospy.get_param('~v_max', default = 0.7)
    wait_for_sport_sole_msg = rospy.get_param('~wait_for_sport_sole_msg', default=False)

    if mode != 'straight':
        rospy.logerr('Unknown mode %s' % mode)
        return

    # Define stage
    stage = 0
    
    # Create Transform Listener
    listener = tf.TransformListener()

    # Create cmd_vel publisher
    pub_cmd_vel = rospy.Publisher('/cmd_vel', geometry_msgs.msg.Twist, queue_size=1)
    msg_cmd_vel = geometry_msgs.msg.Twist()

    # Wait for sport_sole msg
    if wait_for_sport_sole_msg:
        rospy.loginfo('Waiting for sport_sole messages...')
        rospy.wait_for_message('/sport_sole_publisher/sport_sole', sport_sole.msg.SportSole)
        rospy.loginfo('sport_sole message received!')

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        # Update the distance traveled
        if 'trans1' in locals():
            dx = trans[0] - trans1[0]
            dy = trans[1] - trans1[1]
            dth = math.atan2(dy, dx)
            distance_inc = math.hypot(dx, dy)
            distance += distance_inc * math.fabs(math.cos(dth - rot[2]))
        else:
            distance = 0
        (trans1, rot1) = (trans, rot)
        # rospy.loginfo('Distance:  %f; x = %f, y = %f.' % (distance, trans[0], trans[1]) )

        # Calculate the desired velocity
        if stage == 0:
            # Move forward
            e = max(terminal_distance - distance, 0.0)
            kp = 0.8
            u = clamp( math.sqrt( kp * 2.0 * 0.3 * e), 0.0, v_max)
            if e < 0.01:
                u = 0.0
                stage += 1
                t0 = rospy.Time.now()
        elif stage == 1:
            # Wait for 10 secs
            duration_waited = rospy.Time.now() - t0
            if duration_waited.to_sec() > 10.0:
                stage += 1
                distance = 0
        elif stage == 2:
            # Move backward
            e = max(terminal_distance - distance, 0.0)
            kp = 0.8
            u = -clamp( math.sqrt( kp * 2.0 * 0.3 * e), 0.0, v_max)
            if e < 0.01:
                u = 0.0
                stage += 1
        else:
            #Time to shutdown
            pass

        # Populate the message and publish
        msg_cmd_vel.linear.x = u
        pub_cmd_vel.publish(msg_cmd_vel)

        # Terminate if error becomes smaller than tolerance
        if stage > 2:
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