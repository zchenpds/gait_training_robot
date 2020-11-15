#!/usr/bin/env python

# This utility syncs the insole clock with the Kinect IMU lock.
# Oscillating angular motion is needed in order for the timeshift to be estimated.
# To use this utility, hold the insole and Kinect together, wait until the initialization is finished, 
# manually generate the angular velocity oscillation such that the average over a 2-second window is
# larger than 0.5 rad/s

# Limitation: the insoleFreq is manually set in the program and must match the real frequency of the insole data packat,
# Otherwise this program will not calculate the correct time shift.

MIN_ANGULAR_VELOCITY = 0.5
import rospy
import rospkg
import math
import tf
import geometry_msgs.msg
import sport_sole.msg
import sensor_msgs.msg
import message_filters
from scipy import signal
import numpy as np


# Cache length in seconds
cacheLength = 2.0

timeWindow = 3.0
insoleFreq = 145.0

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

    delay_arr = np.linspace(-0.5*n/insoleFreq, 0.5*n/insoleFreq, n)
    delay = delay_arr[np.argmax(corr)]
    rospy.loginfo('Insole timestamp is ' + str(delay) + ' ahead of Kinect IMU timestamp')

def main():
    rospy.init_node('sync_kinect_and_insole')

    # Wait for sport_sole msg
    rospy.loginfo('Waiting for Kinect IMU messages...')
    rospy.wait_for_message('/imu', sensor_msgs.msg.Imu)
    rospy.loginfo('imu message received.')
    rospy.loginfo('Waiting for sport_sole messages...')
    rospy.wait_for_message('/sport_sole_publisher/sport_sole', sport_sole.msg.SportSole)
    rospy.loginfo('sport_sole message received!')

    subKinect = message_filters.Subscriber('/imu', sensor_msgs.msg.Imu)
    cacheKinect = message_filters.Cache(subKinect, 1647 * cacheLength)

    subInsole = message_filters.Subscriber('/sport_sole_publisher/sport_sole', sport_sole.msg.SportSole)
    cacheInsole = message_filters.Cache(subInsole, 100 * cacheLength)
    
    pubWMag = rospy.Publisher('w_mag', geometry_msgs.msg.Vector3Stamped, queue_size=100)


    wKinect = []
    wInsole = []

    rospy.sleep(cacheLength)

    rate = rospy.Rate(insoleFreq)
    while not rospy.is_shutdown():
        t1 = cacheInsole.getOldestTime()
        t2 = cacheInsole.getLatestTime()
        t = t1 + (t2 - t1) / 2
        msgInsole = cacheInsole.getElemAfterTime(t)
        wi = getNorm(msgInsole.angular_velocity[0])
        wInsole.append(wi)

        t = msgInsole.header.stamp

        # Angular velocity of Kinect
        wk = getNorm(cacheKinect.getElemAfterTime(t).angular_velocity)
        wKinect.append(wk)


        msgWMag = geometry_msgs.msg.Vector3Stamped()
        msgWMag.header.stamp = t
        msgWMag.vector.x = wi
        msgWMag.vector.y = wk
        msgWMag.vector.z = 0.0
        pubWMag.publish(msgWMag)

        if len(wKinect) > timeWindow * insoleFreq:
            lag_finder(np.array(wKinect), np.array(wInsole))
            wKinect = []
            wInsole = []


        # Prepare for next iteration
        rate.sleep()
        



if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass