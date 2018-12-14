#!/usr/bin/env python

import rospy
import math
import time
import psutil
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import numpy as np
import scipy.misc as smp
from PIL import Image, ImageDraw

WALL = 1
EMPTY = 0
UNKNOWN = -1

def closeImages():
	for proc in psutil.process_iter():
		if proc.name() == "display":
			proc.kill()

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

	return Map(map_, resolution, pose)

class Map():
	#The permanent map?
	def __init__(self, map_, resolution, pose): #Size is a tuple of [x,y] in meters.
		self.resolution = resolution
		self.map_ = map_
		self.origin = (int(round(len(self.map_)/2)), int(round(len(self.map_[0])/2)))
		self.pose = pose #[x, y, theta], is always relative.  It's the measure of distance from origin.

	def stitch(self, scan_map):
		#Take current map, reference to origin.  Compare to scan, reference to pose.
		#Transform (only location) scan to have the same orientation as origin, and merge.
		new_size = self.bound_check(scan_map)

		new_map = create_empty_map((new_size[2] - new_size[0], new_size[3] - new_size[1]), self.resolution)
		for x in range(int(round(((new_size[2]/self.resolution) - (new_size[0]/self.resolution))))):
			for y in range(int(round(((new_size[3]/self.resolution) - (new_size[1]/self.resolution))))):
				priority = -1

				#This grabs the cell position for each of the two individual graphs of a certain x or y coordinate in the larger graph
				current_map_x = int(round(self.origin[0] + ((new_size[0] + x*self.resolution) - self.pose[0])/self.resolution))
				current_map_y = int(round(self.origin[1] + ((new_size[1] + y*self.resolution) - self.pose[1])/self.resolution))

				scan_map_x = int(round(scan_map.origin[0] + ((new_size[0] + x*scan_map.resolution) - scan_map.pose[0])/scan_map.resolution))
				scan_map_y = int(round(scan_map.origin[1] + ((new_size[1] + y*scan_map.resolution) - scan_map.pose[1])/scan_map.resolution))

				if len(self.map_) > current_map_x and len(self.map_[0]) > current_map_y and current_map_x > 0 and current_map_y > 0 and self.map_[current_map_x][current_map_y] > priority:
					priority = self.map_[current_map_x][current_map_y]

				if len(scan_map.map_) > scan_map_x and len(scan_map.map_[0]) > scan_map_y and scan_map_x > 0 and scan_map_y > 0 and scan_map.map_[scan_map_x][scan_map_y] > priority:
					priority = scan_map.map_[scan_map_x][scan_map_y]

				new_map[x][y] = priority

		self.origin = (int(round((self.pose[0] - new_size[0])/self.resolution)), int(round((self.pose[1] - new_size[1])/self.resolution)))
		self.map_ = new_map


	def bound_check(self, scan_map): #checks if new scan is in bounds, expands map if not.
		#size of the square-map scan is len(map), integer coordinates.

		#Check the sign of the difference in distance to determine where to expand the map if needed.

		lower_x = min(self.pose[0] - self.resolution*(len(self.map_)/2), scan_map.pose[0] - scan_map.resolution*(len(scan_map.map_)/2))
		upper_x = max(self.pose[0] + self.resolution*(len(self.map_)/2), scan_map.pose[0] + scan_map.resolution*(len(scan_map.map_)/2))

		lower_y = min(self.pose[1] - self.resolution*(len(self.map_[0])/2), scan_map.pose[1] - scan_map.resolution*(len(scan_map.map_[0])/2))
		upper_y = max(self.pose[1] + self.resolution*(len(self.map_[0])/2), scan_map.pose[1] + scan_map.resolution*(len(scan_map.map_[0])/2))

		return (lower_x, lower_y, upper_x, upper_y)
		

	def show_map(self):
		# Create a 1024x1024x3 array of 8 bit unsigned integers
		display = Image.new('RGB', (len(self.map_), len(self.map_[0])))
		pixels = display.load()

		for x in range(len(self.map_)):
			for y in range(len(self.map_[0])):
				reading = self.map_[x][y]
				pixels[x, y] = ((0, 0, 0) if (reading == WALL) else ((255, 255, 255) if (reading == EMPTY) else (100, 100, 100)))
				if reading == WALL:

		display.save('small.png')
		display.show()

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
		self.display = None

	def get_odom(self, msg):
		self.pose = convert_pose_to_xy_and_theta(msg.pose.pose)

	def get_scan(self, msg):
		if self.get_new_scan:
			self.get_new_scan = False
			self.scan = msg.ranges

			if self.map_ == None:
				self.map_ = map_from_scan(self.scan, self.pose, self.resolution, self.max_scan)
			else:
				self.map_.stitch(map_from_scan(self.scan, self.pose, self.resolution, self.max_scan))
				closeImages()
			
			self.map_.show_map()

	def run(self):
		go = 0
		while not rospy.is_shutdown():
			self.get_new_scan = True
			time.sleep(5)
			continue

if __name__ == "__main__":
  slammer = Slammer()
  slammer.run()
