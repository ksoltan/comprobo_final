import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray

from markov_model import MarkovModel
from State import State
from Action import Action
import math
import numpy as np
import mdptoolbox # Must install mdptoolbox following documentation.

'''
    Use mdptoolbox to solve the mdp created from markov_model.
    The policy saves action idxs.

'''
class MDPToolbox(object):
    def __init__(self, num_positions=1000, num_orientations=10, map=None, grid_debug=False):
        # Build the markov model
        self.markov_model = MarkovModel(num_positions=num_positions, num_orientations=num_orientations, map=map)
        self.markov_model.make_states(grid_debug=grid_debug)
        self.markov_model.build_roadmap()
        self.num_states = num_positions * num_orientations # S
        if(grid_debug):
            self.num_states = self.markov_model.num_states
        # Visualization
        self.turn_right_pub = rospy.Publisher('/right_pose_array', PoseArray, queue_size=10)
        self.turn_left_pub = rospy.Publisher('/left_pose_array', PoseArray, queue_size=10)
        self.forward_pub = rospy.Publisher('/forward_pose_array', PoseArray, queue_size=10)
        self.goal_state_pub = rospy.Publisher('/goal_state_marker', Marker, queue_size=10)

    '''
        Function: get_policy
        Inputs: State goal_state
                int reward_radius

        Uses mdptoolbox Policy iteration to generate a policy to reach the
        specified goal, within the specified radius.
        Returns policy as tuple with len S, num_iterations, and CPU time

    '''
    def get_policy(self, goal_state, reward_radius=0.15):
        transition_probabilities = self.markov_model.roadmap # Shape (A, S, S)
        goal_state_idx = self.get_goal_state_idx(goal_state)
        print("Goal state idx: {}".format(goal_state_idx))
        rewards = self.get_rewards(goal_state_idx, reward_radius) # Shape (S,)

        discount = 0.9 # How much weight to give to future values
        pi = mdptoolbox.mdp.PolicyIteration(transition_probabilities, rewards, discount)
        pi.run()

        return pi.policy, pi.iter, pi.time

    """
        Function: get_goal_state_idx
        Inputs: State state

        Gets the closest existing state in the roadmap to the goal.
        Returns the goal_state_idx.
    """
    def get_goal_state_idx(self, state):
        print(self.markov_model.get_closest_state_idx(state))
        return self.markov_model.get_closest_state_idx(state)

    '''
        Function: get_rewards
        Inputs: int goal_state_idx
                int reward_radius

        Generates a vector assigning a low reward to all states not within the
        minimum radius around the goal. The high reward is assigned to all
        states that are within this radius, such as multiple orientations of
        the same state.
        Returns an array of shape (S,) assigning a reward to being in each state.

    '''
    def get_rewards(self, goal_state_idx, reward_radius=0.15):
        high_reward = 10
        low_reward = -1

        rewards = np.empty((self.num_states,))
        goal_state = self.markov_model.model_states[goal_state_idx]

        for state_idx in range(self.num_states):
            s = self.markov_model.model_states[state_idx]
            if goal_state.distance_to(s) < reward_radius:
                rewards[state_idx] = high_reward
            else:
                rewards[state_idx] = low_reward
        return rewards

    '''
        Function: visualize_policy
        Inputs: array policy
                State goal_state

        Publishes a sphere marker at the goal state.
        Generates three pose arrays corresponding to the three different actions.
        Depending on which action the policy says to execute at a state, the
        pose of this state is added to the corresponding pose array.

    '''
    def visualize_policy(self, policy, goal_state):
        print("Visualizing policy")
        goal_state_idx = self.get_goal_state_idx(goal_state)
        # Visualize the goal state as sphere.
        self.goal_state_pub.publish(self.markov_model.model_states[goal_state_idx].get_marker())

        turn_left_array = PoseArray()
        turn_right_array = PoseArray()
        forward_array = PoseArray()

        turn_left_array.header.frame_id = "map"
        turn_right_array.header.frame_id = "map"
        forward_array.header.frame_id = "map"

        all_actions = Action.get_all_actions()
        # Add pose of each state to array wih corresponding policy action
        for state_idx in range(len(policy)):
            action = all_actions[policy[state_idx]]
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
    mdp = MDPToolbox(num_positions=100, num_orientations=1)
    print("model.map.info: {}".format(mdp.markov_model.map.info))
    print("Validate is_collision_free - should be False: {}".format(mdp.markov_model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(mdp.markov_model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    # mdp.markov_model.print_states()
    goal_state = State(x=1, y=1, theta=math.radians(40))
    policy, iter, time = mdp.get_policy(goal_state)
    print("Converged: iter: {}, time: {}".format(iter, time))

    while not rospy.is_shutdown():
        r = rospy.Rate(0.5)
        mdp.visualize_policy(policy, goal_state)
        r.sleep()
