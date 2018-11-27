#!/usr/bin/env python

from nav_msgs.msg import OccupancyGrid
import rospy

class Slammer():

	def __init__(self):
		rospy.init_node("square", disable_signals=True)
		rospy.Subscriber('/map', OccupancyGrid, self.getMap)
		self.rate = rospy.Rate(10)

	def getMap(self, map_):
		print(map_.info)

	def run(self):
		while not rospy.is_shutdown():
			continue

if __name__ == "__main__":
  slammer = Slammer()
  slammer.run()