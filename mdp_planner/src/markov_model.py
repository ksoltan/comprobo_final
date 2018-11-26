import rospy
from nav_msgs.srv import GetMap

import numpy as np
import math

class State(object):
    def __init__():
        x = None
        y = None
        theta = None
        # index = None # TODO: Make sure this is always correct
        pass

    def distance(other_state):
        pass



class MarkovModel(object):
    def __init__(self):
        rospy.init_node("markov_model")
        self.model_states = []

        self.roadmap = #[[start_state_idx, end_state_idx, action, probability], ...]

        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

'''
    Function: make_states
    Inputs: int num_states

    Generates a set of states across a given map.
    States are represented as x, y coordinate tuples.
    States should not generate collisions with obstacles in the map.
    Overwrites the current set of self.model_states.

'''
    def make_states(self, num_states=5):
        self.model_states = []
        # TODO: Use state object here, #include orientation

        map_x_range = (self.map.info.origin.position.x, self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution)
        map_y_range = (self.map.info.origin.position.y, self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution)

        print("ranges")
        print(map_x_range)
        print(map_y_range)

        while len(self.model_states) < num_states:
            sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))

            # Add state only if it does not generate a collision.
            if self.is_collision_free(sampled_position):
                self.model_states.append(sampled_position)

        print("model_states")
        print(self.model_states)

'''
    Function: is_collision_free
    Inputs: tuple point
            robot_radius

    Return whether a circle centered at the point with a radius of robot_radius
    generates a collision with an obstacle in the map.
    Only checks a given number of points along the circumference of the circle.

'''
    def is_collision_free(self, point, robot_radius=0.15):
        num_circle_points = 25
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
        # Depends on generate_sample_transitions(), a state, an action, state_distance
        pass

    def generate_sample_transitions():
        pass

    def is_collision_free_path():
        # TODO: probably fine for small enough motion, but should probalby implement a raytrace type thing.
        return True

if __name__ == "__main__":
    model = MarkovModel()
    print(model.map.info)
    model.make_states()
    print(model.is_collision_free((0.97926, 1.4726)))  # Hit wall in ac109_1
    print(model.is_collision_free((1.2823, 1.054)))  # free in ac109_1
