#!/usr/bin/env python

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from helper_functions import *
import rospy
from neato_node.msg import Encoders
from collections import OrderedDict
import cPickle as pickle
from odom_benchmark.cfg import TangoPoseConfig
from dynamic_reconfigure.server import Server
from math import cos, sin

class OdometryBenchmarker(object):
    def __init__(self):
        rospy.init_node('odometry_benchmarker')
        rospy.on_shutdown(self.dump_data)
        srv = Server(TangoPoseConfig, self.config_callback)

        rospy.Subscriber('/STAR_pose', PoseStamped, self.process_groundtruth_pose)
        rospy.Subscriber('/tango_pose', PoseStamped, self.process_tango_pose)
        rospy.Subscriber('/odom', Odometry, self.process_odom)
        rospy.Subscriber('/encoders', Encoders, self.process_encoders)

        self.ground_truth_poses = OrderedDict()
        self.encoders = OrderedDict()
        self.odom_poses = OrderedDict()
        self.tango_poses = OrderedDict()

    def config_callback(self, config, level):
        self.phase_offset = config.phase_offset
        self.pose_correction = config.pose_correction
        return config

    def process_tango_pose(self, m):
        x, y, theta = convert_pose_to_xy_and_theta(m.pose)
        # the tango's pose is flipped the way that I currently have it installed

        print self.pose_correction
        print self.phase_offset
        x += self.pose_correction*cos(theta+self.phase_offset)
        y += self.pose_correction*sin(theta+self.phase_offset)

        self.tango_poses[m.header.stamp.to_sec()] =  x, y, theta

    def process_groundtruth_pose(self, m):
        x, y, theta = convert_pose_to_xy_and_theta(m.pose)
        self.ground_truth_poses[m.header.stamp.to_sec()] =  x, y, theta

    def process_odom(self, m):
        x, y, theta = convert_pose_to_xy_and_theta(m.pose.pose)
        self.odom_poses[m.header.stamp.to_sec()] = x, y, theta

    def process_encoders(self, m):
        self.encoders[m.stamp] = m.leftWheel, m.rightWheel

    def dump_data(self):
        f = open('odom_validation.pickle', 'w')
        pickle.dump(self.ground_truth_poses, f)
        pickle.dump(self.odom_poses, f)
        pickle.dump(self.tango_poses, f)
        pickle.dump(self.encoders, f)
        f.close()

    def run(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = OdometryBenchmarker()
    node.run()