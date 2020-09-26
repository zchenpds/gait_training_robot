#!/usr/bin/env python
import rosbag
import rospy
import rospkg
import os
import progressbar


rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/new/'
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(28, 158)]]

new_suffix = 'l'

def main():
    bar = progressbar.ProgressBar(maxval=len(bag_names), \
        widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
    bar.start()
    i = 0
    for bag_name in bag_names:
        i += 1
        bar.update(i)
        try:
            copy_bags(bag_name, bag_name + new_suffix)
        except:
            print('Cannot open bag: ' + bag_name)
    bar.finish()

def copy_bags(in_bag_name, out_bag_name):
    in_bag_path = path + in_bag_name + '.bag'
    out_bag_path = path + out_bag_name + '.bag'
    
    os.system('cp ' + in_bag_path + ' ' + out_bag_path)

if __name__ == '__main__':
    main()