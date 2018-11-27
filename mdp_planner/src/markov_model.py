import rospy
from nav_msgs.srv import GetMap
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import PoseArray

import numpy as np
import math

from Action import Action
from State import State
'''
    Class: MarkovModel

    Stores a roadmap of all possible states of the robot and the transitions
    between states for any given action.

'''
class MarkovModel(object):
    def __init__(self):
        rospy.init_node("markov_model")

        self.num_states = 100
        self.num_orientations = 10 # number of orientations of one state position to generate
        self.num_transition_samples = 100 # how many transitions to simulate when building roadmap

        self.model_states = [] # All possible positions and orientations
        # Roadmap stores indeces of the states in self.model_states
        self.roadmap = [] #[[start_state_idx, end_state_idx, action, probability], ...]

        # Load map
        # Run: `rosrun map_server map_server ac109_1.yaml` <-indicate yaml file of the map
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

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
    def make_states(self):
        self.model_states = []

        map_x_range = (self.map.info.origin.position.x, self.map.info.origin.position.x + self.map.info.width * self.map.info.resolution)
        map_y_range = (self.map.info.origin.position.y, self.map.info.origin.position.y + self.map.info.height * self.map.info.resolution)

        while len(self.model_states) < self.num_states:
            sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))

            # Add state only if it does not generate a collision.
            if self.is_collision_free(sampled_position):
                # Add multiple orientations
                for i in range(self.num_orientations):
                    self.model_states.append(State(x=sampled_position[0],
                                                   y=sampled_position[1],
                                                   theta=np.random.uniform(0, 2 * math.pi)))

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

    '''
        Function: build_roadmap
        Inputs:

        Set self.roadmap to a list of tuples encoding the probability of
        transitions between a start and end state given an action.
        States are represented as idxs in self.model_states.

    '''
    def build_roadmap(self, num_samples=100):
        for start_state_idx in range(len(self.model_states)):
            for action in Action.get_all_actions():
                end_state_idxs, probabilities = self.get_transitions(start_state_idx, action, self.num_transition_samples)
                # print(zip(end_state_idxs, probabilities))
                # [start_state_idx, end_state_idx, action, probability]
                map_elems = [(start_state_idx, end_state_idxs[i], action, probabilities[i]) for i in range(len(end_state_idxs))]
                if(self.roadmap == []):
                    self.roadmap = np.array(map_elems)
                else:
                    self.roadmap = np.vstack((self.roadmap, map_elems))
        # print(self.roadmap)

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
    def get_transitions(self, start_state_idx, action, num_samples=10):
        # TODO: include a dedicated Obstacle state?
        # TODO: penalize obstacle in path between start and end states
        transitions = {}
        for sample_num in range(0, num_samples):
            sample_state = self.generate_sample_transition(start_state_idx, action)
            end_state_idx = self.get_closest_state_idx(sample_state)
            # print("End_state_idx: {}".format(end_state_idx))

            # Update the probability, or add new state
            if end_state_idx in transitions:
                transitions[end_state_idx] += 1.0
            else:
                transitions[end_state_idx] = 1.0

        return (transitions.keys(), [v / num_samples for v in transitions.values()])

    '''
        Function: get_closest_state_idx
        Inputs: State sample_state

        Return the index of state in self.model_states with
        minimum distance to sample_state.

    '''
    def get_closest_state_idx(self, sample_state):
        # TODO: Re-implement with kd-tree
        min_distance = np.inf
        closest_state_idx = -1

        for state_idx in range(len(self.model_states)):
            distance = sample_state.distance(self.model_states[state_idx])
            if(distance < min_distance):
                min_distance = distance
                closest_state_idx = state_idx
                # print("Found smaller distance: {}, State: {}".format(distance, state_idx))
        return closest_state_idx

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
        pos_sd = 0.1
        theta_sd = 0.1

        start_state = self.model_states[start_state_idx]
        start_x = start_state.x
        start_y = start_state.y
        start_theta = start_state.theta

        linear, angular = Action.get_pose_change(action)

        end_x = start_x + np.random.normal(linear * math.cos(start_theta), pos_sd)
        end_y = start_y + np.random.normal(linear * math.sin(start_theta), pos_sd)
        end_theta = np.random.normal(start_theta + angular, theta_sd) % (2 * math.pi)

        return State(x=end_x, y=end_y, theta=end_theta)

    def is_collision_free_path(self, start_state_idx, end_state_idx):
        # TODO: probably fine for small enough motion, but should implement a raytrace type thing.
        return True

    def print_states(self):
        for i in self.model_states:
            print(i)

    '''
        Function: visualize_roadmap
        Inputs: string Filter - Choose from START_STATE, END_STATE, ACTION. Parameter by which to sort roadmap
                int filter_value - start_state_idx, end_state_idx, or action depending on Filter

        Displays all transitions given a specific start state, end state, or action
    '''
    def visualize_roadmap(self, filter="START_STATE", filter_value=0):
        marker_arr = MarkerArray()
        pose_arr = PoseArray()
        pose_arr.header.frame_id = "map"

        count = 0
        for transition in self.roadmap:
            # Select what to display based on input:\
            start_pose, start_marker, end_pose, end_marker, arrow_marker = None, None, None, None, None
            if(filter == "START_STATE" and transition[0] == filter_value):
                start_pose, start_marker, end_pose, end_marker, arrow_marker = \
                        self.get_transition_markers(int(transition[0]), int(transition[1]), transition[2], transition[3])
            elif(filter == "END_STATE" and transition[1] == filter_value):
                start_pose, start_marker, end_pose, end_marker, arrow_marker = \
                        self.get_transition_markers(int(transition[0]), int(transition[1]), transition[2], transition[3])
            elif(filter == "ACTION" and transition[2] == filter_value):
                start_pose, start_marker, end_pose, end_marker, arrow_marker = \
                        self.get_transition_markers(int(transition[0]), int(transition[1]), transition[2], transition[3])
            if(start_pose != None):
                start_marker.id = count
                marker_arr.markers.append(start_marker)
                count += 1

                end_marker.id = count
                marker_arr.markers.append(end_marker)
                count += 1

                arrow_marker.id = count
                marker_arr.markers.append(arrow_marker)
                count += 1

                pose_arr.poses.append(start_pose)
                pose_arr.poses.append(end_pose)

        # Publish the pose and marker arrays
        print("Num_poses: {}".format(len(pose_arr.poses)))
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

        vector_marker = start_state.get_distance_vector(end_state)

        return (start_state.get_pose(), start_state.get_marker(), end_state.get_pose(),
                end_state.get_marker(r=1.0-probability, g=0.0, b=probability, scale=0.2), vector_marker)

    def clear_visualization(self):
        marker_arr = MarkerArray()
        pose_arr = PoseArray()
        pose_arr.header.frame_id = "map"
        self.state_pose_pub.publish(pose_arr)
        self.marker_pub.publish(marker_arr)

    def solve_mdp(self):
        # Policy (pi) = function of state -> action. Start with random policy.
        #
        # value function (V_pi) = sum of rewards for states to goal, function of state -> sum reward
        #               V_pi(S) = Reward at S + gamma * sum [over_transition_states_S'] (P(S' | S, pi(S))*V_pi(S')) where P(S' | S, pi(S)) is the transition probability
        #
        # in vector from: value_function_vector = rewards_vector + gamma * transition_matrix * value_function_vector
        #                            V_pi       =       R        + gamma *         P_pi      *           V_pi
        # Where P_pi = [
        #               [transition from S1 to S1 with action pi(S1), ..., transition from S1 to Sn with action pi(S1)],
        #               [transition from S2 to S1 with action pi(S2), ..., transition from S2 to Sn with action pi(S1)],
        #                 ...
        #               [transition from Sn to S1 with action pi(Sn), ..., transition from Sn to Sn with action pi(Sn)]
        #              ]    
        #
        # Solve for V_pi with:
        #       V_pi = (I - gamma * P_pi)^-1 * R
        # 
        # Policy'(S) = argmax over actions (R(S) + gamma * sum over transition states S' (transition probability * V_pi'(S')))

        # # # # # # # # # # # # #

        # Make transition matrix
        # Get rewards vector
        # Get Gamma

        # while not has_converged:
        #     P_pi = double loop over states, get value from roadmap
        #     V_pi = inverse(I - gamma * P_pi) * R

        #     has_converged = True
        #     for each state: 
        #         P_state = [
        #             [transition prob from S0 to S with action 0, ..., transition prob from Sn to S with action 0],
        #             ...
        #             [transition prob from S0 to S with action m, ..., transition prob from Sn to S with action m],
        #         ]
        #         R_state = [R(S), ..., R(S)] with length m
        #         new pi(S) = R_state + gamma * V_pi
                
        #         if new pi(S) != pi (S):
        #             has_converged = False

        #     pi = new_pi

        # return pi
        
        pass

if __name__ == "__main__":
    model = MarkovModel()
    print("model.map.info: {}".format(model.map.info))
    model.make_states()
    print("Validate is_collision_free - should be False: {}".format(model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    model.build_roadmap()
    model.clear_visualization()

    while not rospy.is_shutdown():
        r = rospy.Rate(0.5)
        # model.visualize_roadmap(filter="ACTION", filter_value=Action.FORWARD)
        # model.visualize_roadmap(filter="END_STATE", filter_value=50)
        model.visualize_roadmap(filter="START_STATE", filter_value=30)
        r.sleep()

    # for i in range(10):
    #     s = model.generate_sample_transition(0, Action.FORWARD)
    #     print(s)
    #     model.get_closest_state_idx(s)
