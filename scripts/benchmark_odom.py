#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from helper_functions import *
import rospy
from collections import OrderedDict
import cPickle as pickle
import numpy as np
import matplotlib.pyplot as plt
import sys

class OdometryEvaluator(object):
    def __init__(self, pickle_file='odom_validation.pickle'):
        self.use_tango = True
        rospy.init_node('odometry_evaluator')
        f = open(pickle_file)
        self.ground_truth_poses = pickle.load(f)
        self.odom_poses = pickle.load(f)
        self.tango_poses = pickle.load(f)
        f.close()
        self.offset = self.get_odom_ground_truth_offset()

    def get_odom_ground_truth_offset(self):
        if self.use_tango:
            initial_ground_truth_time = self.tango_poses.keys()[0]
            self.initial_ground_truth_pose = self.tango_poses[initial_ground_truth_time]
        else:
            initial_ground_truth_time = self.ground_truth_poses.keys()[0]
            self.initial_ground_truth_pose = self.ground_truth_poses[initial_ground_truth_time]

        initial_odom_pose = self.get_closest_odom_pose(initial_ground_truth_time)
        return (self.initial_ground_truth_pose[0] - initial_odom_pose[0],
                self.initial_ground_truth_pose[1] - initial_odom_pose[1],
                self.initial_ground_truth_pose[2] - initial_odom_pose[2])

    def get_closest_odom_pose(self, timestamp):
        offsets = [abs(t - timestamp) for t in self.odom_poses.keys()]
        best_offset = np.argmin(offsets)
        return self.odom_poses[self.odom_poses.keys()[best_offset]]

    def plot_data(self):
        if self.use_tango:
            ground_truth_times = np.asarray(self.tango_poses.keys())
            ground_truth_poses = np.asarray(self.tango_poses.values())
        else:
            ground_truth_times = np.asarray(self.ground_truth_poses.keys())
            ground_truth_poses = np.asarray(self.ground_truth_poses.values())

        ground_truth = np.hstack((ground_truth_times[:,np.newaxis], ground_truth_poses))
        # ground_truth_poses has an arbitrary angle... need to point forward for something more sensible

        odom_times = np.asarray(self.odom_poses.keys())
        odom_poses = np.asarray(self.odom_poses.values())
        odom = np.hstack((odom_times[:,np.newaxis], odom_poses))

        odom[:,1] += self.offset[0]
        odom[:,2] += self.offset[1]
        shifted = np.vstack((odom[:,1] - self.initial_ground_truth_pose[0],
                             odom[:,2] - self.initial_ground_truth_pose[1]))
        shifted_rotated = shifted.T.dot(np.asarray([[np.cos(-self.offset[2]), -np.sin(-self.offset[2])],
                                                    [np.sin(-self.offset[2]), np.cos(-self.offset[2])]]))

        # TODO: need to define the heading also
        unshifted_rotated = np.vstack((shifted_rotated[:,0] + self.initial_ground_truth_pose[0],
                                       shifted_rotated[:,1] + self.initial_ground_truth_pose[1])).T
        skip_factor = 1
        plt.scatter(ground_truth[::skip_factor,1], ground_truth[::skip_factor,2])
        plt.scatter(unshifted_rotated[:,0], unshifted_rotated[:,1],color='r')
        skip_factor = 1
        plt.axes().set_aspect('equal', 'datalim')
        plt.show()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    if len(sys.argv) == 2:
        node = OdometryEvaluator(sys.argv[1])
    else:
        node = OdometryEvaluator()
    node.plot_data()
    node.run()