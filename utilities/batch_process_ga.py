#!/usr/bin/env python
import rospkg
import os

rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/optitrack/'
bag_number_list = range(213, 230)
suffix = 'b'

for bag_number in bag_number_list:
    bag_name = 'data' + str(bag_number).zfill(3) + suffix

    os.system('roslaunch gait_training_robot ga.launch record_bag:=false \
    play_bag:=true bag_name:=optitrack/' + bag_name + ' enable_gait_analyzer:=true record_gait_analytics:=true')

    for file in os.listdir(path):
        if bag_name in file and '_ga' in file:
            # print(file)
            i_cutoff = file.rfind('_ga')
            os.system('mv ' + path + file + ' ' + path + file[0:(i_cutoff+3)] + '.bag')