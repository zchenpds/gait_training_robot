#!/usr/bin/python

# Shut down the system if the publish rate is too low

import rospy
import actionlib
from std_msgs.msg import String
from rostopic import ROSTopicHz
import rostopic
import os

min_rate_body_tracking = 20.0
min_rate_sport_sole = 200.0

wait_duration_secs = 14.0

class PublishRateMonitor():
    def __init__(self, topic_name, min_rate):
        self.topic_name_ = topic_name
        self.min_rate_ = min_rate
        self.rt_ = ROSTopicHz(self.min_rate_ * 4)
        self.sub_ = rospy.Subscriber(topic_name, rospy.AnyMsg, self.rt_.callback_hz)

    def check(self):
        hz = self.rt_.get_hz()
        if not hz:
            rospy.logerr(self.topic_name_ + ': no data')
            return False
        elif hz[0] < self.min_rate_:
            rospy.logerr(self.topic_name_ + ': ' + '%.2f' % hz[0] + ' Hz < ' + '%.2f' % self.min_rate_ + ' Hz' )
            return False
        return True
    


loop_rate = 1

def main():
    rospy.init_node('record_monitor', anonymous=True)
    rate = rospy.Rate(loop_rate)

    prms = [PublishRateMonitor('/body_tracking_data', min_rate_body_tracking), \
            PublishRateMonitor('/sport_sole_publisher/sport_sole', min_rate_sport_sole)]

    t = 0.0
    while not rospy.is_shutdown():
        if t > wait_duration_secs:
            if any( [not prm.check() for prm in prms] ):
                if t < wait_duration_secs + 5:
                    # Stop the goal_generator only if the robot has not moved far
                    os.system('rosnode kill -a')
        rate.sleep()
        t += 1.0 / loop_rate

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


