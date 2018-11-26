import rospy
from nav_msgs.srv import GetMap

import numpy as np
import math

'''
    Class: Action

    Enumeration of possible robot actions. The dynamics of each action is
    specified in MarkovModel's generate_sample_transition.
'''
class Action:
    LEFT = -1
    FORWARD = 0
    RIGHT = 1

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
        # index = None # TODO: Make sure this is always correct

    '''
        Function: distance
        Inputs: State other

        Returns the weighted distance between two states,
        including both position and orientation.
    '''
    def distance(self, other):
        alpha = 1 # TODO: Check how to weight relative orientation and distance
        return math.sqrt((self.x - other.x)**2 +
                         (self.y - other.y)**2 +
                         alpha*(self.theta - other.theta)**2)

    def __str__(self):
        return "State: x, y, theta - ({}, {}, {} deg)".format(self.x, self.y, math.degrees(self.theta))

'''
    Class: MarkovModel

    Stores a roadmap of all possible states of the robot and the transitions
    between states for any given action.
'''
class MarkovModel(object):
    def __init__(self):
        rospy.init_node("markov_model")
        self.model_states = [] # All possible positions and orientations

        # Roadmap stores indeces of the states in self.model_states
        self.roadmap = [] #[[start_state_idx, end_state_idx, action, probability], ...]

        # Load map
        # Run: `rosrun map_server map_server ac109_1.yaml` <-indicate yaml file of the map
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

    '''
        Function: make_states
        Inputs: int num_states

        Generates a set of states across a given map.
        States should not generate collisions with obstacles in the map.
        Overwrites the current set of self.model_states.

    '''
    def make_states(self, num_states=20, num_orientations=4):
        self.model_states = []

        map_x_range = (self.map.info.origin.position.x, self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution)
        map_y_range = (self.map.info.origin.position.y, self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution)

        while len(self.model_states) < num_states:
            sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))

            # Add state only if it does not generate a collision.
            if self.is_collision_free(sampled_position):
                # Add multiple orientations
                for i in range(num_orientations):
                    self.model_states.append(State(x=sampled_position[0],
                                                   y=sampled_position[1],
                                                   theta=np.random.uniform(0, 2 * math.pi)))

        self.print_states()

    '''
        Function: is_collision_free
        Inputs: tuple point - x, y
                robot_radius
                num_circle_points - More points increases resolution of collision-check

        Return whether a circle centered at the point with a radius of robot_radius
        generates a collision with an obstacle in the map.
        Only checks a given number of points along the circumference of the circle.

    '''
    def is_collision_free(self, point, robot_radius=0.15, num_circle_points=25):
        for angle in np.linspace(0, 2 * math.pi, num_circle_points):
            circle_point = (robot_radius * math.cos(angle) + point[0], robot_radius * math.sin(angle) + point[1])
            x_coord = int((circle_point[0] - self.map.info.origin.position.x) / self.map.info.resolution)
            y_coord = int((circle_point[1] - self.map.info.origin.position.y) / self.map.info.resolution)

            is_occupied = self.map.data[x_coord + self.map.info.width * y_coord]

            if is_occupied:
                return False

        return True

    def build_roadmap():
        # Depends on isCollisionFree(), self.states, getTransitions()
        pass

    def get_transitions():
        # Depends on generate_sample_transition(), a state, an action, state_distance
        pass

    def generate_sample_transition():
        pass

    def is_collision_free_path():
        # TODO: probably fine for small enough motion, but should implement a raytrace type thing.
        return True

    def print_states(self):
        for i in self.model_states:
            print(i)

if __name__ == "__main__":
    model = MarkovModel()
    print(model.map.info)
    model.make_states()
    print(model.is_collision_free((0.97926, 1.4726)))  # Hit wall in ac109_1
    print(model.is_collision_free((1.2823, 1.054)))  # free in ac109_1
