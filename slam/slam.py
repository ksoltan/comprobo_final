#!/usr/bin/env python

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

WALL = 1
EMPTY = 0
UNKNOWN = -1

def create_empty_map(size, resolution):
	map_ = []
	for x in range(int(round(size[0]/resolution))):
		map_.append([])
		for y in range(int(round(size[1]/resolution))):
			map_[x].append(UNKNOWN)

	return map_

def round_to_resolution(num, resolution):
	return round(num * (1/resolution)) / (1/resolution)

def convert_pose_to_xy_and_theta(pose):
	""" Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
	orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
	angles = euler_from_quaternion(orientation_tuple)
	return (pose.position.x, pose.position.y, angles[2])

def map_from_scan(scan, pose, resolution, max_scan):
	map_ = create_empty_map([max_scan*2, max_scan*2], resolution)
	for angle in range(360):

		end_dist = (min(scan[angle], max_scan) if scan[angle] != 0.0 else max_scan)

		for dist in range(int(round(end_dist/resolution))):
			x = (dist*resolution)*math.cos(pose[2] + math.radians(angle))
			y = (dist*resolution)*math.sin(pose[2] + math.radians(angle))
			map_[round(x/resolution)][round(y/resolution)] = (WALL if map[x][y] == WALL else EMPTY)

		if end_dist != max_scan:
			map_[x][y] = WALL

	return Map(map_, pose)

class Map():
	def __init__(self, map_, pose):
		self.map_ = map_
		self.pose = pose

	def stitch(self, map):
		#Take current map, reference to origin.  Compare to scan, reference to pose.
		#Transform scan to have the same orientation as origin, and 
		pass

class Slammer():
	def __init__(self):
		rospy.init_node("slam")
		self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.get_scan)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)

		self.rate = rospy.Rate(10)
		self.scan = []
		self.get_new_scan = True
		self.max_scan = 3
		self.resolution = 0.1
		self.pose = (0, 0, 0)
		self.map_ = None

	def get_odom(self, msg):
		self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

	def get_scan(self, msg):
		if self.get_new_scan:
			self.scan = msg.ranges
			self.map_ = map_from_scan(self.scan, self.pose, self.resolution, self.max_scan)
			print self.map_
			self.get_new_scan = False

	def run(self):
		while not rospy.is_shutdown():
			continue

if __name__ == "__main__":
  slammer = Slammer()
  slammer.run()
