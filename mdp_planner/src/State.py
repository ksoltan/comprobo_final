import math
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Vector3, Pose
from Action import Action
'''
    Class: State

    A state represents the possible position (x, y) and orientation (theta)
    of the robot in the map.

'''
class State(object):
    def __init__(self, x=None, y=None, theta=None):
        self.x = x
        self.y = y
        self.theta = theta # Radians

    '''
        Function: distance_between
        Inputs: tuple pose1
                tuple pose2

        Returns the weighted distance between two poses which are tuples of
        (x, y, theta). Used in the BallTree computation.
    '''
    # Made sense when trying to call the function from kd-tree
    # @staticmethod
    # def distance_between(pose1, pose2, alpha=1):
    #     # TODO: Check how to weight relative orientation and distance
    #     return math.sqrt((pose1[0] - pose2[0])**2 +
    #                      (pose1[1] - pose2[1])**2 +
    #                      alpha * (math.pi - abs(abs(pose1[2] - pose2[2]) - math.pi)))  # Wraparound magic
    
    # def distance_to(self, other):
    #     return State.distance_between(self.get_pose_xytheta(), other.get_pose_xytheta())

    @staticmethod
    def angle_diff(angle, other):
        return math.pi - abs(abs(angle - other) - math.pi)  # Wraparound magic

    def distance_to(self, other, alpha=1):
        # TODO: Check how to weight relative orientation and distance
        return math.sqrt((self.x - other.x)**2 +
                         (self.y - other.y)**2 +
                         alpha * self.angle_diff(self.theta, other.theta))

    def get_pose_xytheta(self):
        return (self.x, self.y, self.theta)

    '''
        Function: distance_to
        Inputs: State other

        Returns the weighted distance between two states,
        including both position and orientation.

    '''

    def get_marker(self, r=0.0, g=1.0, b=0.0, scale=0.15, lifetime=2):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"

        marker.type = Marker.SPHERE
        marker.color.a = 0.7
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b

        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.lifetime = rospy.Time(lifetime)

        return marker

    def get_pose(self):
        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        # Euler to Quaternion angle
        pose.orientation.w = math.cos(self.theta / 2.0)
        pose.orientation.z = math.sin(self.theta / 2.0)

        return pose

    def get_distance_vector(self, other, action=Action.FORWARD):
        marker = Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"

        marker.type = Marker.ARROW
        marker.color.a = 0.7

        # Match color of transition to the action.
        if(action == Action.RIGHT):
            marker.color.r = 1.0
            marker.color.b = 1.0
        elif(action == Action.LEFT):
            marker.color.r = 1.0
        else:
            marker.color.g = 1.0
        marker.scale.x = 0.02
        marker.scale.y = 0.05
        marker.scale.z = 0.02

        marker.points = [Vector3(self.x, self.y, 0), Vector3(other.x, other.y, 0)]
        marker.lifetime = rospy.Time(2)

        return marker

    def __str__(self):
        return "State: x, y, theta - ({}, {}, {} deg)".format(self.x, self.y, math.degrees(self.theta))
