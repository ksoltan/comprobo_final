#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy.misc as smp

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
			map_[int(round(x/resolution + len(map_)/2))][int(round(y/resolution + len(map_[0])/2))] = (WALL if (map_[int(round(x/resolution))][int(round(y/resolution))] == WALL) else EMPTY)

		if end_dist != max_scan:
			map_[int(round(x/resolution + len(map_)/2))][int(round(y/resolution + len(map_[0])/2))] = WALL

	return Map(map_, resolution)

class Map():
	#The permanent map?
	def __init__(self, size): #Size is a tuple of [x,y] in meters.
		self.resolution = 0.05
		self.map_ = create_empty_map(size, self.resolution)
		self.size = size
		self.origin = (math.floor(self.size[0]/2), math.floor(self.size[1]/2))
        #self.pose = [] #[x, y, theta], is always relative.  It's the measure of distance from origin.

    def stitch(self, scan_map, pose):
        #Take current map, reference to origin.  Compare to scan, reference to pose.
        #Transform (only location) scan to have the same orientation as origin, and merge.
		self.bound_check(self.map)
		scan_size = len(scan_map)
		#origin + pose = index.
		start_point = [(self.origin[0] + pose[0]-scan_size/2),
		(self.origin[1] + pose[1] + scan_size/2)]

		for i in range(0, scan_size): #compare the scan_map to the scan.
			for j in range(0, scan_size):
				new_scan  = scan_map[i][j]
				reference = self.map[start_point[0] + i][start_point[1] + j] #equivalent point on map
				#based on priority. 1 is a solid wall and always takes priority.
				#0 is navigable space, and -1 is unknown.
				self.map_[start_point[0] + i][start_point[1] + j] = max(new_scan, reference)


	def bound_check(self, map_): #checks if new scan is in bounds, expands map if not.
		#size of the square-map scan is len(map), integer coordinates.

		#Check the sign of the difference in distance to determine where to expand the map if needed.

		dist_x, dist_y = round(self.pose[0])/resolution,
		round(self.pose[1])/resolution #convert distance to integer coordinates.
		mapsize_x = self.size[0]
		mapsize_y = self.size[1]
		#Map expansion works by creating a new map with the expanded size.
		#First it determines if the map needs to be expanded by looking at
		# if the bot is within the current map.  Then it calculates the
		#amount that needs to be expanded, then determines
		#if the origin needs to be translated (if negative), since the map can only
		#be expanded by adding indices.
		map_expand_x = 0
		map_expand_y = 0
		translate_origin_x = 0
		translate_origin_y = 0

		#Checks if the map is within boundary, and if not, it will expand
		if (dist_x >= 0): #If right from origin
			if (dist_x + (len(map_)/2) > mapsize_x/2):
				map_expand_x = (dist_x)
		if (dist_y >= 0): #If above origin.
			if (dist_y + (len(map_)/2) > mapsize_y/2):
				map_expand_y = dist_y
		if (dist_x <= 0):
			if (abs(dist_x - (len(map_)/2)) > mapsize_x/2):
				map_expand_x = abs(dist_x)
				translate_origin_x = map_expand_x #only need to translate origin if robot expands map left/down
		if (dist_y <= 0):
			if (abs(dist_y - (len(map_)/2)) > mapsize_y/2):
				map_expand_y = abs(dist_y)
				translate_origin_y = map_expand_y

		#now to translate the origin
		self.origin = (self.origin[0] + translate_origin_x, self.origin[1] + translate_origin_y)
		#update size
		self.size = [mapsize_x + map_expand_x, mapsize_y + map_expand_y]
		#actually expand map
		

	def show_map():
		# Create a 1024x1024x3 array of 8 bit unsigned integers
		display = np.zeros((len(self.map_), len(self.map_[0]), 3), dtype=np.uint8)

		for x in range(len(self.map_)):
			for y in range(len(self.map_[0])):
				reading = self.map_[x][y]
				display[x][y] = ([0, 0, 0] if (reading == WALL) else ([255, 255, 255] if (reading == EMPTY) else [100, 100, 100]))

		img = smp.toimage(display)
		img.show()

class Slammer():
	def __init__(self):
		rospy.init_node("slam")
		self.scan_subscriber = rospy.Subscriber('scan', LaserScan, self.get_scan)
		self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)

		self.rate = rospy.Rate(10)
		self.scan = []
		self.get_new_scan = True
		self.max_scan = 3
		self.resolution = 0.01
		self.pose = (0, 0, 0)
		self.map_ = None
		self.mainMap = None

	def get_odom(self, msg):
		self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

	def get_scan(self, msg):
		if self.get_new_scan:
			self.scan = msg.ranges
			self.map_ = map_from_scan(self.scan, self.pose, self.resolution, self.max_scan)
			#self.map_.show_map()


			if self.mainMap == None:
				self.mainMap = self.map_
			else:
				self.mainMap.stitch(self.map_, self.pose)
				self.get_new_scan = False
				self.mainMap.show_map()

	def run(self):
		while not rospy.is_shutdown():
			continue

if __name__ == "__main__":
  slammer = Slammer()
  slammer.run()
