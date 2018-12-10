import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray

from markov_model import MarkovModel
from State import State
from Action import Action
import math
import numpy as np
# import mdptoolbox # Must install mdptoolbox following documentation.

'''
    Class: MDP

    Builds the optimal policy for a given markov model and a specified goal
    to define what action the robot should take in any given state to reach
    the goal.

'''
class MDP(object):
    def __init__(self, num_positions=1000, num_orientations=10, map=None):
        # Build the markov model
        self.markov_model = MarkovModel(num_positions=num_positions, num_orientations=num_orientations, map=map)
        self.markov_model.make_states()
        self.markov_model.build_roadmap()

        self.num_states = num_positions * num_orientations

        self.goal_state_idx = 0
        self.reward_radius = 0
        self.rewards = np.empty(self.num_states)

        self.policy = [] # List of actions to take corresponding to each state idx in markov_model.model_states
        self.value_function = [] # List of rewards corresponding to being each state based on the policy

        # Visualization
        self.turn_right_pub = rospy.Publisher('/right_pose_array', PoseArray, queue_size=10)
        self.turn_left_pub = rospy.Publisher('/left_pose_array', PoseArray, queue_size=10)
        self.forward_pub = rospy.Publisher('/forward_pose_array', PoseArray, queue_size=10)
        self.goal_state_pub = rospy.Publisher('/goal_state_marker', Marker, queue_size=10)

    """
        Function: set_goal_state
        Inputs: State state
                int reward_radius

        Sets the state that the robot wants to end up in, as well as the
        radius around this state which will count as close enough.
    """
    def set_goal_state(self, state, reward_radius=0.5):
        self.goal_state_idx = self.markov_model.get_closest_state_idx(state)
        # print(self.goal_state_idx)
        # print(self.markov_model.model_states[self.goal_state_idx])
        self.reward_radius = reward_radius

    '''
        Function: set_rewards
        Inputs:

        Generates a vector assigning a low reward to all states not within the
        minimum radius around the goal. The high reward is assigned to all
        states that are within this radius, such as multiple orientations of
        the same state.

    '''
    def set_rewards(self):
        high_reward = 10
        low_reward = -1

        goal_state = self.markov_model.model_states[self.goal_state_idx]
        for state_idx in range(self.num_states):
            s = self.markov_model.model_states[state_idx]
            if goal_state.distance_to(s) < self.reward_radius:
                self.rewards[state_idx] = high_reward
            else:
                self.rewards[state_idx] = low_reward

    """
        Function: make_policy
        Inputs: State goal_state - new goal state to be set.

        Calculates the optimal policy for a given (or saved) goal state by
        iteratively solving a value function (how much payoff each action at
        a state will have in the long run) to update the action to take in each
        state (policy).

    """
    def make_policy(self, goal_state=None):
        if(goal_state != None):
            self.set_goal_state(goal_state)

        # Generate reward vector
        self.set_rewards()

        # Generate a starting, random policy from all available actions
        self.policy = self.get_random_policy()

        change = -1
        iteration_count = 0
        while change != 0:
            print("iteration {}".format(iteration_count))
            iteration_count += 1
            # Find V of this policy.
            solved = self.solve_value_function()
            if(solved):
                # Calculate new policy.
                change = self.get_new_policy()
                print(change)
            else:
                print("Failed to solve for a value function, trying a new random policy")
                # Singular matrix solution to V
                self.policy = self.get_random_policy()
        print("Converged!")
        print(self.policy)

    """
        Function: get_random_policy
        Inputs:

        Returns a randomly-assigned policy, or action to be taken in each state.

    """
    def get_random_policy(self):
        return np.random.choice(Action.get_all_actions(), self.num_states, replace=True)

    """
        Function: get_new_policy
        Inputs:

        Computes a new policy by looping through each start state and assigning
        the action that generates the highest reward long-term.
        Returns 0 if no changes to the policy were made. Any other integer
        means that different actions were chosen as compared to the previous
        policy.

    """
    def get_new_policy(self):
        gamma = 0.999
        all_actions = Action.get_all_actions()
        total_change = 0
        for state_idx in range(self.num_states):
            ps_matrix = self.build_ps_matrix(state_idx)
            state_rewards = self.rewards[state_idx] + gamma * ps_matrix.dot(self.value_function)
            idx_action = state_rewards.argmax()
            total_change += self.policy[state_idx] - all_actions[idx_action]
            self.policy[state_idx] = all_actions[idx_action]
        return total_change

    """
        Function: solve_value_function
        Inputs:

        Solves for the value function which calculates the infinite horizon or
        payoof at infinite time at each state if following the current policy.
        Returns false if the matrix is non-invertible.

    """
    def solve_value_function(self):
        print("Solving value function")
        # Build P matrix.
        p_matrix = self.build_p_matrix()
        # print(p_matrix)

        I = np.identity(self.num_states)
        gamma = 0.999
        print(p_matrix.sum(axis=1))
        if(np.linalg.det(I - gamma * p_matrix) == 0):
            return False
        self.value_function =  np.linalg.inv(I - gamma * p_matrix).dot(self.rewards)
        return True

    """
        Function: build_p_matrix
        Inputs:

        Returns a num_states x num_states matrix with each element representing
        the probability of transitioning from a start_state to end_state with
        the action defined in the policy.

    """
    def build_p_matrix(self):
        print("Building P matrix")
        p_matrix = np.empty([self.num_states, self.num_states])
        for start_state_idx in range(self.num_states):
            for end_state_idx in range(self.num_states):
                action = self.policy[start_state_idx]
                p_matrix[start_state_idx][end_state_idx] = \
                        self.markov_model.get_probability(start_state_idx, end_state_idx, action)
        return p_matrix

    """
        Function: build_ps_matrix
        Inputs:

        Returns a num_actions x num_states matrix with each element representing
        the probability of transitioning from a start_state to end_state with
        the action specified in each row.

    """
    def build_ps_matrix(self, state_idx):
        print("Building PS matrix")
        all_actions = Action.get_all_actions()
        ps_matrix = np.empty([len(all_actions), self.num_states])
        for i in range(len(all_actions)):
            action = all_actions[i]
            for j in range(self.num_states):
                start_state_idx = j
                ps_matrix[i][j] = self.markov_model.get_probability(start_state_idx, state_idx, action)
        return ps_matrix

    def visualize_policy(self):
        # Visualize the goal state as sphere.
        self.goal_state_pub.publish(self.markov_model.model_states[self.goal_state_idx].get_marker())

        turn_left_array = PoseArray()
        turn_right_array = PoseArray()
        forward_array = PoseArray()

        turn_left_array.header.frame_id = "map"
        turn_right_array.header.frame_id = "map"
        forward_array.header.frame_id = "map"

        # Add pose of each state to array wih corresponding policy action
        for state_idx in range(len(self.policy)):
            action = self.policy[state_idx]
            state_pose = self.markov_model.model_states[state_idx].get_pose()

            if(action == Action.LEFT):
                turn_left_array.poses.append(state_pose)
            elif(action == Action.RIGHT):
                turn_right_array.poses.append(state_pose)
            else:
                forward_array.poses.append(state_pose)

        self.turn_left_pub.publish(turn_left_array)
        self.turn_right_pub.publish(turn_right_array)
        self.forward_pub.publish(forward_array)

if __name__ == "__main__":
    mdp = MDP(num_positions=100, num_orientations=1)
    print("model.map.info: {}".format(mdp.markov_model.map.info))
    print("Validate is_collision_free - should be False: {}".format(mdp.markov_model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(mdp.markov_model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    mdp.markov_model.print_states()
    mdp.set_goal_state(State(x=1, y=1, theta=math.radians(40)))
    mdp.set_rewards()
    print(mdp.rewards)
    mdp.make_policy()

    while not rospy.is_shutdown():
        r = rospy.Rate(0.5)
        mdp.visualize_policy()
        r.sleep()
