#! /usr/bin/env python3

from distutils.log import error
from time import time
from turtle import distance
from mpl_toolkits.mplot3d import Axes3D
from numpy.core.fromnumeric import shape
import rosbag
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
from scipy.interpolate import interp1d
from matplotlib.ticker import AutoMinorLocator, FormatStrFormatter
import matplotlib.ticker as ticker
from scipy.signal import medfilt
from matplotlib import cbook
from mpl_toolkits.axes_grid1.inset_locator import zoomed_inset_axes
from mpl_toolkits.axes_grid1.inset_locator import mark_inset
from mpl_toolkits.axisartist.parasite_axes import HostAxes, ParasiteAxes
from matplotlib.ticker import MaxNLocator
import matplotlib.animation
from pylab import *
from numpy import genfromtxt


def main():
    bags_path = "../bags/"
    bags = [bags_path + "replayed_fog_filled_uni_corridor.bag"]
    norm_of_whole_dist_maes = []
    final_drifts_percent = []
    total_travelled_dists = []
    arrays_of_maes = []
    arrays_of_ttds = []

    for bag in bags:
        with rosbag.Bag(bag, 'r') as imubag:
            radar_position = np.empty(shape=(0, 3), dtype=np.float64)
            radar_attitude = np.empty(shape=(0, 4), dtype=np.float64)
            # real_time = np.empty(shape=(0, 1), dtype=np.float64)
            real_radar_time = np.empty(shape=(0, 1), dtype=np.float64)
            pose_real_time = np.empty(shape=(0, 1), dtype=np.float64)
            # imu_position = np.empty(shape=(0, 3), dtype=np.float64)
            imu_pos = np.empty(shape=(0, 3), dtype=np.float64)
            # imu_attitude = np.empty(shape=(0, 4), dtype=np.float64)
            pos_error_var = np.empty(shape=(0, 3), dtype=np.float64)
            ori_error_var = np.empty(shape=(3, 3), dtype=np.float64)
            pos_error_cov = np.empty(shape=(3, 3), dtype=np.float64)
            calib_pos = np.empty(shape=(0, 3), dtype=np.float64)
            calib_ori = np.empty(shape=(0, 4), dtype=np.float64)
            state_time = np.empty(shape=(0, 1), dtype=np.float64)

            for topic, msg, t in imubag.read_messages():
                if topic == "/pose" and msg.pose:
                    imu_pos = np.append(imu_pos, np.asarray(
                        [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=np.float32)[np.newaxis, :], axis=0)
                    pose_real_time = np.append(pose_real_time, np.asarray(
                        [msg.header.stamp.secs + 1e-9*msg.header.stamp.nsecs], dtype=np.float64)[np.newaxis, :], axis=0)
                    ori_error_var = np.append(ori_error_var, np.asarray(
                        [[msg.pose.covariance[21], msg.pose.covariance[22], msg.pose.covariance[23]],
                         [msg.pose.covariance[27], msg.pose.covariance[28],
                            msg.pose.covariance[29]],
                         [msg.pose.covariance[33], msg.pose.covariance[34], msg.pose.covariance[35]]], dtype=np.float32), axis=0)
                    pos_error_cov = np.append(pos_error_cov, np.asarray(
                        [[msg.pose.covariance[0], msg.pose.covariance[1], msg.pose.covariance[2]],
                         [msg.pose.covariance[6], msg.pose.covariance[7],
                            msg.pose.covariance[8]],
                         [msg.pose.covariance[12], msg.pose.covariance[13], msg.pose.covariance[14]]], dtype=np.float32), axis=0)
                    pos_error_var = np.append(pos_error_var, np.asarray(
                        [msg.pose.covariance[0], msg.pose.covariance[7], msg.pose.covariance[14]], dtype=np.float32)[np.newaxis, :], axis=0)

        # Normalize time vectors to count from zero and make vectors 1D.
        # real_time = np.squeeze(real_time - real_time[0])
        pose_real_time = np.squeeze(pose_real_time - pose_real_time[0])
        # Choose the smallest end timestamp from all time signals.
        t_end = pose_real_time[-1]

        # Get the difference in time at the begininning of logging
        # to account for ofset.

        # Choose timestamp from which the filter is considered to have converged.
        # In seconds.
        t_conv = 0
        # t_conv = 26

        # Move position to zero (estimate already starts from zero).
        # radar_position = radar_position - radar_position[0]

        imu_pos[:, :] = imu_pos[:, :] - imu_pos[0, :]

        # Interpolate signals from zero up to the common t_end.
        # Interpolation is needed to compute errors (MAE) and clip
        # signals to converged phase.
        imu_pos_interp = interp1d(
            pose_real_time, imu_pos, kind='nearest', axis=0)

        # Create common timespan from zero up until t_end.
        timespan = np.linspace(
            t_conv, t_end, pose_real_time.size, endpoint=True)

        # Clip timespan to chosen initial value.
        imu_pos_clipped = imu_pos_interp(timespan)
        # Compute the offset between signals at t_conv.

        # Uncomment to align data.
        # imu_pos_clipped = imu_pos_clipped + \
        #    np.absolute(radar_pos_clipped[0, :] - imu_pos_clipped[0, :])

        # Signals only for visualizing trajectories wrt waypoints.
        # Assign clipped arrays to ones we plot.
        imu_position_vis = imu_pos_clipped

        # Compute errors and drifts.

        # Append to all drifts and maes.

        # Below 2d position plots.
        fig0, ax0 = plt.subplots(3, sharex=True)
        fig0.suptitle('RIO estimated position vs ground truth', fontsize=50)
        plot0, = ax0[0].plot(
            timespan, imu_pos_clipped[:, 0], 'b', label="RIO")
        ax0[0].set_ylabel('x [m]', fontsize=50)
        ax0[0].yaxis.set_minor_locator(AutoMinorLocator())
        ax0[0].tick_params(axis='both', labelsize=35)
        ax0[0].grid(True)
        ax0[0].legend(loc='lower left', fontsize=25)
        plot0, = ax0[1].plot(
            timespan, imu_pos_clipped[:, 1], 'b', label="RIO")
        ax0[1].set_ylabel('y [m]',  fontsize=50)
        ax0[1].yaxis.set_minor_locator(AutoMinorLocator())
        ax0[1].tick_params(axis='both', labelsize=35)
        ax0[1].grid(True)
        plot0, = ax0[2].plot(
            timespan, imu_pos_clipped[:, 2], 'b', label="RIO")
        ax0[2].set_xlabel('time [s]',  fontsize=50, labelpad=-1)
        ax0[2].set_ylabel('z [m]',  fontsize=50)
        ax0[2].set_xlim([t_conv, t_end])
        ax0[2].yaxis.set_minor_locator(AutoMinorLocator())
        ax0[2].xaxis.set_minor_locator(AutoMinorLocator())
        ax0[2].tick_params(axis='both', labelsize=35)
        ax0[2].grid(True)
        fig0.align_ylabels(ax0)

        # Below plot 3D ground truth trajectory and beacons.
        imu_position_vis = imu_position_vis.T
        fig2 = plt.figure()
        ax2 = fig2.add_subplot(111, projection='3d')
        fig2.suptitle(
            '3D trajectory of the UAV', fontsize=60)
        ax2.set_xlabel('x [m]', fontsize=50, labelpad=60)
        ax2.set_ylabel('y [m]', fontsize=50, labelpad=60)
        ax2.set_zlabel('z [m]', fontsize=50, labelpad=60)
        ax2.tick_params(axis='x', labelsize=42, pad=20)
        ax2.tick_params(axis='y', labelsize=42, pad=20)
        ax2.tick_params(axis='z', labelsize=42, pad=20)

        ax2.plot(imu_position_vis[0], imu_position_vis[1],
                 imu_position_vis[2], marker='+', label='UAV trajectory')

        ax2.scatter(*imu_position_vis.T[0],
                    color='yellow', label='initial UAV position', s=140)
        ax2.scatter(*imu_position_vis.T[-1],
                    color='black', label='final UAV position', s=140)

        # 2D projection of trajectory.
        fig3 = plt.figure()
        ax3 = fig3.add_subplot(111)
        fig3.suptitle(
            'Trajectory of the UAV projected onto x-y', fontsize=45)
        ax3.set_xlabel('x [m]', fontsize=50, labelpad=-1)
        ax3.set_ylabel('y [m]', fontsize=50)
        ax3.tick_params(axis='x', labelsize=33)
        ax3.tick_params(axis='y', labelsize=33)
        ax3.grid(True)

        ax3.plot(imu_position_vis[0], imu_position_vis[1],
                 marker='+', label='UAV trajectory')

        ax3.plot(imu_position_vis.T[0, 0], imu_position_vis.T[0, 1],
                 marker='o', color='blue', label='initial UAV position', markersize=20)
        ax3.plot(imu_position_vis.T[-1, 0], imu_position_vis.T[-1, 1],
                 marker='o', color='black', label='final UAV position', markersize=20)
        plt.show(block=False)

    plt.show()


if __name__ == '__main__':
    main()
