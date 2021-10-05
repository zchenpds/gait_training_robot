#!/usr/bin/env python
import rospkg
import os

rp = rospkg.RosPack()
outbag_path = rp.get_path('gait_training_robot') + '/bags/optitrack/odom/'
bag_number_list = range(213, 214)

for bag_number in bag_number_list:
    bag_name = 'data' + str(bag_number).zfill(3)

    for file in os.listdir(outbag_path):
        if bag_name in file:
            print('Deleting ' + file)
            os.remove(outbag_path + file)

    os.system('roslaunch gait_training_robot play.launch record_odom:=true \
        bag_name:=' + bag_name)

    for file in os.listdir(outbag_path):
        if bag_name in file:
            # print(file)
            os.system('mv ' + outbag_path + file + ' ' + outbag_path + file[0:7] + '.bag')
            print('Created ' + outbag_path + file[0:7] + '.bag')