#!/usr/bin/env python

import rospy
from math import *
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

WALL = 1
EMPTY = 0
UNKNOWN = -1

def create_empty_map(size, resolution): #creates an empty occupancy grid
	map_ = []
	for x in round(range(-(size[0]/resolution)/2), round((size[0]/resolution)/2)):
		for y in range(round(-(size[1]/resolution)/2), round((size[1]/resolution)/2)):
			map_[x][y] = UNKNOWN

	return map_

def round_to_resolution(num, resolution):
	return round(num * (1/resolution)) / (1/resolution)

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

def map_from_scan(map_, scan, pose, resolution, max_scan):
	for angle in range(360):

		end_dist = (min(scan[angle], max_scan) if scan[angle] != 0.0 else max_scan)

		for dist in range(0, end_dist, resolution):
			x = dist*math.cos(math.radians(angle))
			y = dist*math.sin(math.radians(angle))
			map_[x][y] = (WALL if map_[x][y] == WALL else EMPTY)

		if end_dist != max_scan:
			map_[x][y] = WALL

	return map_

class Map():
	def __init__(self, size): #Size is a tuple of [x,y] in meters.
		self.resolution = 0.05
		self.map = create_empty_map(size, self.resolution)
		self.size = size

		self.origin = (0, 0)
		self.map = []
        self.pose = [] #[x, y, theta]

	    def stitch(map_):
	        #Take current map, reference to origin.  Compare to scan, reference to pose.
	        #Transform (only location) scan to have the same orientation as origin, and merge.
			bound_check(map_)

		def bound_check(map_): #checks if new scan is in bounds, expands map if not.
			#size of the square-map scan is len(map), integer coordiinates.
			#Check the sign of the difference in distance to detemrine where to expand the map if needed.

			dist_x, dist_y = round(self.origin[0] - self.pose[0])/resolution,
			round(self.origin[1] - self.pose[1])/resolution) #convert distance to integer coordinates.
			mapsize_x = self.size[0]
			mapsize_y = self.size[1]
			if (dist_x >= 0):
				if dist_x + (len(map_)/2) > mapsize_x/2
				#EXPAND MAP
			if (dist_y >= 0):
				if dist_y + (len(map_)/2) > mapsize_y/2
			if (dist_x <= 0):
				if dist_x + (len(map_)/2) > mapsize_x/2
				#EXPAND MAP
			if (dist_y <= 0):
				if dist_y + (len(map_)/2) > mapsize_y/2


class Slammer():
	def __init__(self):
		rospy.init_node("slam")
		self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.get_scan)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)

        self.rate = rospy.Rate(10)
		self.scan = []
		self.map = Map()
		self.get_new_scan = True
		self.max_scan = 3
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
