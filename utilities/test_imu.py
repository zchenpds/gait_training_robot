#!/usr/bin/python

# roscd gait_training_robot/bags/new/
# rosbag play

import math
import rosbag
import rospy
import rospkg
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal


rp = rospkg.RosPack()
path = rp.get_path('gait_training_robot') + '/bags/new/'
bag_names = ['data' + s for s in [str(i).rjust(3, '0') for i in range(48,49)]]

def main():
    for bag_name in bag_names:
        test_imu(bag_name)
        try:
            # modify_bag(bag_name, bag_name + 'x')
            pass
        except:
            print('Cannot open bag: ' + bag_name)



def test_imu(bag_name):
    bag_path = path + bag_name + '.bag'
    with rosbag.Bag(bag_path, 'r') as inbag:

        # IMU data 
        ts_imu = []
        wz = []
        wy = []
        wx = []
        ax = []
        ay = []
        for topic, msg, ts in inbag.read_messages(topics = '/imu'):
            ts_imu.append(msg.header.stamp.to_sec())
            wz.append(msg.angular_velocity.z)
            wy.append(msg.angular_velocity.y)
            wx.append(msg.angular_velocity.x)
            ax.append(msg.linear_acceleration.x)
            ay.append(msg.linear_acceleration.y)
        ts_imu = np.array(ts_imu)
        wz = -np.array(wz)
        wy = np.array(wy)
        wx = np.array(wx)
        ax = np.array(ax)
        ay = np.array(ay)
        
        # odometry data
        ts_odom = []
        psi = []
        vx = []
        px = []
        for topic, msg, ts in inbag.read_messages(topics = '/odom'):
            ts_odom.append(msg.header.stamp.to_sec())
            z = msg.pose.pose.orientation.z
            psi.append(math.asin(z * 2))
            vx.append(msg.twist.twist.linear.x)
            px.append(msg.pose.pose.position.x)
        ts_odom = np.array(ts_odom)
        psi = np.array(psi)
        vx = np.array(vx)
        px = np.array(px)
        
        # Remove time offset
        ts_offset = ts_imu[0]
        ts_imu -= ts_offset
        ts_odom -= ts_offset

        # Select time
        t_range = [0, 20]
        idx_imu = np.nonzero( (ts_imu < t_range[1]) & (ts_imu > t_range[0]) )
        ts_imu = ts_imu[idx_imu]
        wz = wz[idx_imu]
        wy = wy[idx_imu]
        wx = wx[idx_imu]
        ax = ax[idx_imu]
        ay = ay[idx_imu]
        idx_odom = np.nonzero( (ts_odom < t_range[1]) & (ts_odom > t_range[0]) )
        ts_odom  = ts_odom[idx_odom]
        psi = psi[idx_odom]
        vx = vx[idx_odom]
        px = px[idx_odom]
        
        # Differentiate velocity
        dvx = np.concatenate((np.array([0.]), np.diff(vx) / np.diff(ts_odom)))
        dvx = np.interp(ts_imu, ts_odom, dvx)

        # Remove gyro offset
        wz_offset = np.mean(wz[ts_imu < 4.])
        wz = wz - wz_offset
        wy_offset = np.mean(wy[ts_imu < 4.])
        wy = wy - wy_offset
        wx_offset = np.mean(wx[ts_imu < 4.])
        wx = wx - wx_offset

        # Calculate pitch angle with the projection of gravitational acceleration on the x axis
        theta_acc = np.arctan2(ax - dvx, 9.8) # pitch angle
        phi_acc = np.arctan2(ay, 9.8)
        
        # Integration
        dt = np.concatenate( (np.array([0.]), np.diff(ts_imu)) )
        wz_int = np.cumsum(wz * dt)
        # wx_int
        # wx_int = np.cumsum(wx * dt)
        wx_int = np.zeros(len(ts_imu))
        wx_int[0] = np.mean(phi_acc[ts_imu < 4.])
        wx_dt = wx * dt
        gamma = 1. / 5. / 1600.
        for i in range(1, len(ts_imu)):
            wx_int[i] = wx_int[i - 1] + wx_dt[i]
            wx_int[i] += gamma * (phi_acc[i] - wx_int[i])
        # wy_int
        # wy_int = np.cumsum(wy * dt) + np.mean(theta_acc[ts_imu < 4.])
        wy_int = np.zeros(len(ts_imu))
        wy_int[0] = np.mean(theta_acc[ts_imu < 4.])
        wy_dt = wy * dt
        gamma = 1. / 5. / 1600.
        for i in range(1, len(ts_imu)):
            wy_int[i] = wy_int[i - 1] + wy_dt[i]
            wy_int[i] += gamma * (theta_acc[i] - wy_int[i])

        # Remove acc offset
        ax_offset = 9.8 * np.sin(wy_int)
        ax = ax - ax_offset
        # ax_int = np.cumsum(ax * dt)
        ax_dt = ax * dt
        vx_imu = np.interp(ts_imu, ts_odom, vx)
        px_imu = np.interp(ts_imu, ts_odom, px)
        ax_int = np.zeros(len(ts_imu))
        ax_int2 = np.zeros(len(ts_imu))
        ax_int[0] = np.mean(vx_imu[ts_imu < 4.])
        ax_int2[0] = 0.0
        gamma = 1. / 2. / 1600.
        for i in range(1, len(ts_imu)):
            ax_int[i] = ax_int[i - 1] + ax_dt[i]
            ax_int[i] += gamma * (vx_imu[i] - ax_int[i])
            ax_int2[i] = ax_int2[i - 1] + ax_int[i] * dt[i]

        # Filtering
        wx_filtered = np.zeros(len(wx))
        wx_filtered[0] = 0.0
        gamma = 0.98
        for i in range(1, len(wx)):
            wx_filtered[i] = wx_filtered[i - 1] * gamma + wx[i] * (1 - gamma)

        # # Visualize angular in z
        # plt.subplot(211)
        # plt.plot(ts_imu, wz, label = 'wz')
        # plt.legend()
        # plt.title('Angular velocity measured by the Kinect IMU')
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Angular velocity in z [rad]')

        # plt.subplot(212)
        # plt.plot(ts_odom, psi, label = 'Wheel Odometry')
        # plt.plot(ts_imu, wz_int, label = 'Kinect IMU: psi[i] = psi[i-1] + (wz[i] - wz_offset[i])')
        # plt.legend()
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Angular displacement in z [rad]')
        # plt.title('Angular displacement comparison between wheel odometry and integration of gyroscope signal')
        # plt.show()

        # # Visualize angular in x
        # plt.subplot(211)
        # plt.plot(ts_imu, wx)
        # plt.title('Angular velocity measured by the Kinect IMU')
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Angular Velocity [rad/sec]')
        # plt.subplot(212)
        # plt.plot(ts_imu, phi_acc, label = 'Raw roll angle')
        # plt.plot(ts_imu, wx_int, label = 'Fused roll angle')
        # plt.legend()
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Angular displacement[rad]')
        # plt.title('Angular displacement in x')
        # plt.show()
        
        # spectrum
        plt.subplot(211)
        plt.plot(ts_imu, wx, label = 'Raw')
        plt.plot(ts_imu, wx_filtered, label = 'Filtered')
        plt.legend()
        plt.title('Angular velocity measured by the Kinect IMU')
        plt.xlabel('Time [sec]')
        plt.ylabel('Angular Velocity [rad/sec]')
        plt.subplot(212)
        plt.magnitude_spectrum(wx, 1600, label = 'Raw')
        plt.magnitude_spectrum(wx_filtered, 1600, label = 'Filtered')
        plt.show()

        # Visualize angular in y
        plt.subplot(211)
        plt.plot(ts_imu, wy)
        plt.title('Angular velocity measured by the Kinect IMU')
        plt.subplot(212)
        plt.plot(ts_imu, wy_int, label = 'Kinect IMU')
        plt.xlabel('Time [sec]')
        plt.ylabel('Angular displacement[rad]')
        plt.title('Angular displacement in y')
        plt.show()

        # # Visualize linear in x
        # plt.subplot(311)
        # plt.plot(ts_imu, theta_acc, label = 'Raw pitch angle: theta_acc = atan2(ax - diff(vx) / diff(t), 9.8)')
        # plt.plot(ts_imu, wy_int, label = 'Fused pitch angle: theta_fused')
        # plt.title('Pitch angle calculation using complementary filter')
        # plt.legend()
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Angular displacement in y [rad]')

        # plt.subplot(312)
        # plt.plot(ts_imu, ax_int, label = 'Kinect IMU: vx_fused')
        # plt.plot(ts_odom, vx, label = 'Wheel Odometry')
        # plt.legend()
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Linear velocity [m/s]')
        # plt.title('Linear velocity comparison between wheel odometry and integration of accelerometer signal')

        # plt.subplot(313)
        # # plt.plot(ts_imu, ax_int2, label = 'ax_int2')
        # # plt.plot(ts_imu, px_imu, label = 'px_imu')
        # plt.plot(ts_imu, ax_int2 - px_imu, label = 'difference')
        # plt.legend()
        # plt.xlabel('Time [sec]')
        # plt.ylabel('Displacement x [m]')
        # plt.show()



if __name__ == '__main__':
    main()