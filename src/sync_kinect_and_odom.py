#!/usr/bin/env python

# This utility syncs the insole clock with the Kinect IMU lock.
# Oscillating angular motion is needed in order for the timeshift to be estimated.
# To use this utility, hold the insole and Kinect together, wait until the initialization is finished, 
# manually generate the angular velocity oscillation such that the average over a 2-second window is
# larger than 0.5 rad/s

# Limitation: the odomFreq is manually set in the program and must match the real frequency of the insole data packat,
# Otherwise this program will not calculate the correct time shift.

MIN_ANGULAR_VELOCITY = 0.5
import rospy
import rospkg
import math
import tf
import geometry_msgs.msg
import nav_msgs.msg
import sensor_msgs.msg
import message_filters
from scipy import signal
import numpy as np


# Cache length in seconds
cacheLength = 2.0

timeWindow = 15.0
odomFreq = 10.0

def getNorm(w):
    return math.sqrt(w.x * w.x + w.y * w.y + w.z * w.z)

def lag_finder(y1, y2):
    n = len(y1)
    y1bar = np.average(y1)
    y2bar = np.average(y2)

    if y1bar < MIN_ANGULAR_VELOCITY or y2bar < MIN_ANGULAR_VELOCITY:
        rospy.logwarn('Synchronization failed. Angular velocity is less than ' + str(MIN_ANGULAR_VELOCITY) + ' rad/s.')
        return
    elif math.fabs(y1bar - y2bar) > 0.2:
        rospy.logwarn('Synchronization failed. The difference in average magnitude of angualr velocity between Kinect and insole is too large.')
        return


    corr = signal.correlate(y2, y1, mode='same') / np.sqrt(signal.correlate(y1, y1, mode='same')[int(n/2)] * signal.correlate(y2, y2, mode='same')[int(n/2)])

    delay_arr = np.linspace(-0.5*n/odomFreq, 0.5*n/odomFreq, n)
    delay = delay_arr[np.argmax(corr)]
    rospy.loginfo('Insole timestamp is ' + str(delay) + ' ahead of Kinect IMU timestamp')

def main():
    rospy.init_node('sync_kinect_and_insole')

    # Wait for sport_sole msg
    rospy.loginfo('Waiting for Kinect IMU messages...')
    rospy.wait_for_message('/imu', sensor_msgs.msg.Imu)
    rospy.loginfo('imu message received.')
    rospy.loginfo('Waiting for odom messages...')
    rospy.wait_for_message('/odom', nav_msgs.msg.Odometry)
    rospy.loginfo('odom message received!')

    subKinect = message_filters.Subscriber('/imu', sensor_msgs.msg.Imu)
    cacheKinect = message_filters.Cache(subKinect, 100 * cacheLength)

    subInsole = message_filters.Subscriber('/odom', nav_msgs.msg.Odometry)
    cacheInsole = message_filters.Cache(subInsole, 10 * cacheLength)
    
    pubWMag = rospy.Publisher('w_mag', geometry_msgs.msg.Vector3Stamped, queue_size=100)


    wKinect = []
    wOdom = []

    rospy.sleep(cacheLength)

    rate = rospy.Rate(odomFreq)
    while not rospy.is_shutdown():
        t1 = cacheInsole.getOldestTime()
        t2 = cacheInsole.getLatestTime()
        t = t1 + (t2 - t1) / 2
        msgOdom = cacheInsole.getElemAfterTime(t)
        wi = getNorm(msgOdom.twist.twist.angular)
        wOdom.append(wi)

        t = msgOdom.header.stamp

        # Angular velocity of Kinect
        wk = getNorm(cacheKinect.getElemAfterTime(t).angular_velocity)
        wKinect.append(wk)


        msgWMag = geometry_msgs.msg.Vector3Stamped()
        msgWMag.header.stamp = t
        msgWMag.vector.x = wi
        msgWMag.vector.y = wk
        msgWMag.vector.z = 0.0
        pubWMag.publish(msgWMag)

        if len(wKinect) > timeWindow * odomFreq:
            lag_finder(np.array(wKinect), np.array(wOdom))
            wKinect = []
            wOdom = []


        # Prepare for next iteration
        rate.sleep()
        



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass