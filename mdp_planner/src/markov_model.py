import rospy
from nav_msgs.srv import GetMap
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray

import numpy as np
from sklearn.neighbors import KDTree, BallTree
from collections import OrderedDict
import math
import bisect
import time
import sys

from Action import Action
from State import State
'''
    Class: MarkovModel

    Stores a roadmap of all possible states of the robot and the transitions
    between states for any given action.

'''

def draw_progress_bar(percent, bar_length=30):
    """
    0 < percent < 100
    Draws a a static progress bar.
    """
    scale = bar_length / 100.0
    completed = percent * scale
    remaining = (100 - percent) * scale
    sys.stdout.write("\r[" + "=" * int(round(completed)) + ' ' * int(round(remaining)) + ']   {:4.2f}% '.format(percent))
    sys.stdout.flush()

class MarkovModel(object):
    def __init__(self, num_positions=1000, num_orientations=10, map=None):
        rospy.init_node("markov_model")

        self.num_positions = num_positions
        self.num_orientations = num_orientations # number of orientations of one state position to generate
        self.num_states = self.num_positions * self.num_orientations

        self.num_transition_samples = 20#100 # how many transitions to simulate when building roadmap

        self.model_states = []  # All possible positions and orientations
        self.kd_tree = None

        # Roadmap is a three dim array, axis 0 = start_state_idx, axis 1 = end_state_idx, axis_2 = action idx in list
        self.roadmap = np.zeros([len(Action.get_all_actions()), self.num_states, self.num_states])
        print("Empty roadmap")
        print(self.roadmap)

        # Load map
        # Run: `rosrun map_server map_server ac109_1.yaml` <-indicate yaml file of the map
        if(map == None):
            rospy.wait_for_service("static_map")
            static_map = rospy.ServiceProxy("static_map", GetMap)
            self.map = static_map().map
        else:
            self.map = map

        # Visualization
        self.state_pose_pub = rospy.Publisher('/state_pose_array', PoseArray, queue_size=10)
        self.marker_pub = rospy.Publisher('/marker_array', MarkerArray, queue_size=10)

    '''
        Function: make_states
        Inputs:

        Generates a set of states across a given map.
        States should not generate collisions with obstacles in the map.
        Overwrites the current set of self.model_states.

    '''
    def make_states(self, grid_debug=False):
        self.model_states = []
        self.positions = []
        # np.random.seed(0)
        self.position_to_states = OrderedDict()  # {(x, y): [(angle_1, state_idx_1), (angle_2, state_idx_2), ...]} All with the same position.

        count = 0
        map_x_range = (self.map.info.origin.position.x, self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution)
        map_y_range = (self.map.info.origin.position.y, self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution)
       
        if(grid_debug):
            poses = []
            num_x_pos = int(math.sqrt(self.num_positions * (map_x_range[1] - map_x_range[0]) / (map_y_range[1] - map_y_range[0])))
            num_y_pos = int(self.num_positions / num_x_pos)
            print("{} x {}".format(num_x_pos, num_y_pos))
            for x in np.linspace(map_x_range[0], map_x_range[1], num_x_pos + 10):
                for y in np.linspace(map_y_range[0], map_y_range[1], num_y_pos + 10):
                    for angle in np.linspace(0, 2 * math.pi, self.num_orientations, endpoint=False):
                        if self.is_collision_free((x, y)):
                            self.model_states.append(State(x=x, y=y, theta=angle))
                            poses.append([x, y, angle])
                            print("Num states = {}".format(count))
                            count += 1
            # Re-initialize lists with different state numbers!
            self.num_states = count
            self.roadmap = np.zeros([len(Action.get_all_actions()), self.num_states, self.num_states])
            print(poses)
        else:
            while len(self.model_states) < self.num_states:
                sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))

                # Add state only if it does not generate a collision.
                if self.is_collision_free(sampled_position):
                    orientation_states = []

                    # Add multiple orientations
                    for i in range(self.num_orientations):
                        angle = np.random.uniform(0, 2 * math.pi)
                        self.model_states.append(State(x=sampled_position[0],
                                                    y=sampled_position[1],
                                                    theta=angle))

                        bisect.insort(orientation_states, (angle, count)) # do sorted insert based on angle
                        count += 1

                    self.position_to_states[sampled_position] = orientation_states
                    self.positions.append([sampled_position[0], sampled_position[1]])

        print("Num states = {}".format(len(self.model_states)))

        print("Building KD Tree...")
        start = time.time()
        self.kd_tree = KDTree(self.positions, metric='euclidean')
        print("It took {}s\n".format(time.time()-start))
        # Note: Is this better than doing  a list comprehension on self.model_states memory-wise?

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

            # check if we are in bounds
            # Used in robot_localization/robot_localizer/scripts/occupancy_field.py
            if x_coord > self.map.info.width or x_coord < 0:
                return False
            if y_coord > self.map.info.height or y_coord < 0:
                return False

            ind = x_coord + y_coord * self.map.info.width

            if ind >= self.map.info.width*self.map.info.height or ind < 0:
                return False

            is_occupied = self.map.data[ind]
            if is_occupied:
                return False

        return True

    '''
        Function: build_roadmap
        Inputs:

        Set self.roadmap to a list of tuples encoding the probability of
        transitions between a start and end state given an action.
        States are represented as idxs in self.model_states.

    '''
    def build_roadmap(self, num_samples=100):
        print("Building roadmap.")
        start = time.time()
        num_transitions = 0
        num_actions = len(Action.get_all_actions())

        for action_idx in range(num_actions):
            for start_state_idx in range(len(self.model_states)):

                if start_state_idx % 100 == 0:
                    draw_progress_bar(100.0 * (start_state_idx + (len(self.model_states) * action_idx)) / (len(self.model_states) * num_actions))

                action = Action.get_all_actions()[action_idx]
                transitions = self.get_transitions(start_state_idx, action, self.num_transition_samples)
                # print(zip(end_state_idxs, probabilities))
                # [start_state_idx, end_state_idx, action, probability]

                for end_state_idx, probability in transitions.iteritems():
                    self.roadmap[action_idx][start_state_idx][end_state_idx] = probability
                    
            # print("num transitions = {}".format(transitions))
            num_transitions += 1
        print("\nroadmap took {}s".format(time.time() - start))

    '''
        Function: get_transitions()
        Inputs: int start_state_idx
                Action action
                int num_samples

        Returns a list of all possible end_states and their probabilities from
        the given state and action.
        Returns a tuples of lists (state_idxs, probabilities)
        Simulates num_samples transitions to calculate probabilities

    '''
    def get_transitions(self, start_state_idx, action, num_samples=100):
        # TODO: include a dedicated Obstacle state?
        transitions = {}
        for sample_num in range(0, num_samples):
            target_state = self.generate_sample_transition(start_state_idx, action)
            end_state_idx = self.get_closest_state_idx(target_state, start_state_idx=start_state_idx)

            # Update the probability, or add new state
            if end_state_idx in transitions:
                transitions[end_state_idx] += 1.0 / num_samples
            else:
                transitions[end_state_idx] = 1.0 / num_samples

        return transitions

    '''
        Function: get_closest_state_idx
        Inputs: State target_state

        Return the index of state in self.model_states with
        minimum distance to target_state.

        Uses a KD Tree to find states with near position, then linearly
        searches for the best combination of position and angle.

    '''
    def get_closest_state_idx(self, target_state, start_state_idx=None):
        num_close_state = 5
        distances, closest_position_indeces = self.kd_tree.query(np.array([target_state.get_pose_xytheta()[:2]]), k=num_close_state)
        
        return self.get_closest_orientation_idx(target_state, closest_position_indeces[0], start_state_idx=start_state_idx)

    def get_closest_orientation_idx(self, target_state, closest_position_indeces, start_state_idx=None):
        position = self.positions[closest_position_indeces[0]]
        orientations = self.position_to_states[tuple(position)]

        best_state_idx = None
        distance = float('inf')

        for position_idx in closest_position_indeces:
            position = self.positions[position_idx]
            orientations = self.position_to_states[tuple(position)]
            new_state_idx = self.nearest_angle_state_idx(orientations, target_state.theta)
            new_distance = target_state.distance_to(self.model_states[new_state_idx])

            if new_distance < distance and new_state_idx != start_state_idx:
                distance = new_distance
                best_state_idx = new_state_idx

        if best_state_idx is None:
            print("All checked state have distance > inf, returning starting state. This really shouldn't happen.")
            return start_state_idx

        return best_state_idx

    '''
        Function: generate_sample_transition
        Inputs: int start_state_idx
                Action action

        Returns a State that can be achieved with some probability from the
        given state and action.
        Samples from a normal distribution to approximate result of action.

    '''
    def generate_sample_transition(self, start_state_idx, action):
        # Standard deviations
        # TODO: FIX STANDARD DEVIATIONS. They are not reasonable. However, if they are too small,
        #       then more states need to be generated to have meaningful transitions (not to the state itself)
        pos_sd = 0.01
        theta_sd = 0.1

        start_state = self.model_states[start_state_idx]
        start_x = start_state.x
        start_y = start_state.y
        start_theta = start_state.theta

        linear, angular = Action.get_pose_change(action)
        # print("Lin: {}, ang: {}".format(linear, angular))


        end_x = start_x + np.random.normal(linear * math.cos(start_theta), pos_sd)
        end_y = start_y + np.random.normal(linear * math.sin(start_theta), pos_sd)
        end_theta = np.random.normal(start_theta + angular, theta_sd) % (2 * math.pi)
        # print(State(x=end_x, y=end_y, theta=end_theta))

        return State(x=end_x, y=end_y, theta=end_theta)

    '''
        Function: get_probability
        Input: int start_state_idx
               int end_state_idx
               Action action

        Returns the probability associations with transitioning between the
        start and end state by executing the specified action.

    '''
    def get_probability(self, start_state_idx, end_state_idx, action):
        action_idx = Action.get_all_actions().index(action)
        return self.roadmap[action_idx][start_state_idx][end_state_idx]

    @staticmethod
    def nearest_angle_state_idx(sorted_angle_list, target_angle):
        """
        Find the nearest angle with binary search on sorted list.
        There be python demons here, but it's been thoroughly tested.
        sorted_angle_list = [(angle, state_idx), ...]
        0 < target_angle < 2 * pi

        returns state_idx
        """
        # Put in tuple since the list has tuples. Second item doesn't matter.
        bisect_idx = bisect.bisect(sorted_angle_list, (target_angle, None)) % len(sorted_angle_list)
        # Compare item at bisect_idx and bisect_idx - 1. If former is larger, then choose it, otherwise subtract 1. Wraps around. #pythonbullshit
        best_idx = bisect_idx - (State.angle_diff(sorted_angle_list[bisect_idx][0], target_angle) > State.angle_diff(sorted_angle_list[bisect_idx - 1][0], target_angle))
        return sorted_angle_list[best_idx][1] #, best_idx
        

    def is_collision_free_path(self, start_state_idx, end_state_idx):
        for angle in np.linspace(0, 2 * math.pi, num_circle_points):
            circle_point = (robot_radius * math.cos(angle) + point[0], robot_radius * math.sin(angle) + point[1])
            x_coord = int((circle_point[0] - self.map.info.origin.position.x) / self.map.info.resolution)
            y_coord = int((circle_point[1] - self.map.info.origin.position.y) / self.map.info.resolution)

            # check if we are in bounds
            # Used in robot_localization/robot_localizer/scripts/occupancy_field.py
            if x_coord > self.map.info.width or x_coord < 0:
                return False
            if y_coord > self.map.info.height or y_coord < 0:
                return False

            ind = x_coord + y_coord * self.map.info.width

            if ind >= self.map.info.width*self.map.info.height or ind < 0:
                return False

            is_occupied = self.map.data[ind]
            if is_occupied:
                return False
        # TODO: probably fine for small enough motion, but should implement a raytrace type thing.
        return True

    '''
        Function: print_states
        Input:

        Print all model_states.
    '''
    def print_states(self):
        for i in self.model_states:
            print(i)

    '''
        Function: visualize_roadmap
        Inputs: string Filter - Choose from START_STATE, END_STATE, ACTION. Parameter by which to sort roadmap
                int filter_value - start_state_idx, end_state_idx, or action_idx depending on Filter

        Displays all transitions given a specific start state, end state, or action indices.

    '''
    def visualize_roadmap(self, filter="START_STATE", filter_value=0):
        marker_arr = MarkerArray()
        pose_arr = PoseArray()
        pose_arr.header.frame_id = "map"

        transitions = []
        if(filter == "START_STATE"):
            transitions = self.roadmap[:, filter_value, :]
        elif(filter == "END_STATE"):
            transitions = self.roadmap[:, :, filter_value]
        elif(filter == "ACTION"):
            transitions = self.roadmap[filter_value, :, :]

        print(transitions.shape)

        count = 0
        for i in range(transitions.shape[0]):
            for j in range(transitions.shape[1]):
                if(filter == "START_STATE"):
                    action_idx = i
                    start_state_idx = filter_value
                    end_state_idx = j
                    probability = self.roadmap[action_idx][start_state_idx][end_state_idx]
                elif(filter == "END_STATE"):
                    action_idx = i
                    start_state_idx = j
                    end_state_idx = filter_value
                    probability = self.roadmap[action_idx][start_state_idx][end_state_idx]
                elif(filter == "ACTION"):
                    action_idx = filter_value
                    start_state_idx = i
                    end_state_idx = j
                    probability = self.roadmap[action_idx][start_state_idx][end_state_idx]
                else:
                    continue
                start_pose, start_marker, end_pose, end_marker, arrow_marker = \
                    self.get_transition_markers(start_state_idx, end_state_idx, Action.get_all_actions()[action_idx], probability)

                if(start_pose != None and probability > 0.2):
                    print("Finding viz for start: {}, end: {}, action: {}".format(start_state_idx, end_state_idx, action_idx))

                    if(filter == "END_STATE" or filter == "START_STATE"):
                        start_marker.id = count
                        marker_arr.markers.append(start_marker)
                        count += 1

                    # end_marker.id = count
                    # marker_arr.markers.append(end_marker)
                    # count += 1

                    arrow_marker.id = count
                    marker_arr.markers.append(arrow_marker)
                    count += 1

                    pose_arr.poses.append(start_pose)
                    pose_arr.poses.append(end_pose)

        # Publish the pose and marker arrays
        # print("Num_poses: {}".format(count / 3))
        self.state_pose_pub.publish(pose_arr)
        self.marker_pub.publish(marker_arr)

    '''
        Function: get_transition_markers
        Inputs: int start_state_idx
                int end_state_idx
                Action action
                float probability

        Return a tuple of (start_pose, start_marker, end_pose, end_marker, arrow_marker).
        The start state is a large green sphere.
        The end state is a blue to red color, depending on the probability.
        The arrow_marker points from start to end state.

    '''
    def get_transition_markers(self, start_state_idx, end_state_idx, action, probability):
        start_state = self.model_states[start_state_idx]
        end_state = self.model_states[end_state_idx]

        vector_marker = start_state.get_distance_vector(end_state, action)

        return (start_state.get_pose(), start_state.get_marker(), end_state.get_pose(),
                end_state.get_marker(r=1.0-probability, g=0.0, b=probability, scale=0.1), vector_marker)

    '''
        Function: clear_visualization
        Input:

        Publish empty pose and marker arrays to clear the rviz map.

    '''
    def clear_visualization(self):
        marker_arr = MarkerArray()
        pose_arr = PoseArray()

        pose_arr.header.frame_id = "map"
        self.state_pose_pub.publish(pose_arr)
        self.marker_pub.publish(marker_arr)

if __name__ == "__main__":
    model = MarkovModel(num_positions=1000, num_orientations=10)
    print("model.map.info: {}".format(model.map.info))
    model.make_states()
    print("Validate is_collision_free - should be False: {}".format(model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    model.build_roadmap()
    print(model.roadmap)
    model.clear_visualization()
    # model.print_states()
    # model.visualize_roadmap(filter="START_STATE", filter_value=0)
    # while not rospy.is_shutdown():
    #     r = rospy.Rate(0.5)
    #     # model.visualize_roadmap(filter="START_STATE", filter_value=0)
    #     # model.visualize_roadmap(filter="ACTION", filter_value=Action.get_all_actions().index(Action.LEFT))
        
    #     model.visualize_roadmap(filter="ACTION", filter_value=Action.get_all_actions().index(Action.FORWARD))
        
    #     # model.visualize_roadmap(filter="ACTION", filter_value=Action.get_all_actions().index(Action.RIGHT))
    #     # model.visualize_roadmap(filter="END_STATE", filter_value=4)
    #     # model.visualize_roadmap(filter="START_STATE", filter_value=0)
    #     r.sleep()
