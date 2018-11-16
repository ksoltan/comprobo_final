import rospy
from nav_msgs.srv import GetMap

import numpy as np
import math

class MarkovModel(object):
    def __init__(self):
        rospy.init_node("markov_model")
        self.model_states = []
        
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

    def make_states(self, num_states=5):
        map_x_range = (self.map.info.origin.position.x, self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution)
        map_y_range = (self.map.info.origin.position.y, self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution)

        print("ranges")
        print(map_x_range)
        print(map_y_range)

        while len(self.model_states) < num_states:
            sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))
            
            if self.is_collision_free(sampled_position):
                self.model_states.append(sampled_position)

        print("model_staets")
        print(self.model_states)

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

if __name__ == "__main__":
    model = MarkovModel()
    print(model.map.info)
    model.make_states()
    print(model.is_collision_free((0.97926, 1.4726)))  # Hit wall in ac109_1
    print(model.is_collision_free((1.2823, 1.054)))  # free in ac109_1