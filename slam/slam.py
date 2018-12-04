#!/usr/bin/env python

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def map_from_scan(map, scan, pose, resolution, max_scan):
	for angle in range(360):
		if scan[angle] > max_scan or scan[angle] = 0.0:
			for x in range(0, max_scan*sin(angle))

	return map

class Map():
	def __init__(self):
		self.resolution = 0.05
		self.origin = (0, 0)
		self.map = []

class Slammer():
	def __init__(self):
		rospy.init_node("slam")
		self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.get_scan)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)
		self.rate = rospy.Rate(10)
		self.scan = []
		self.map = Map()
		self.get_new_scan = True
		self.max_scan = 2.5
		self.pose = (0, 0, 0)

	def get_odom(self, msg):
		self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

	def get_scan(self, msg):
		if self.get_new_scan:
			self.scan = msg.ranges
			self.get_new_scan = False

	def run(self):
		while not rospy.is_shutdown():
			continue

if __name__ == "__main__":
  slammer = Slammer()
  slammer.run()