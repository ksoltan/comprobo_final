'''
    Class: MDP

    Builds the optimal policy for a given markov model and a specified goal
    to define what action the robot should take in any given state to reach
    the goal.

'''
from markov_model import MarkovModel
from State import State
from Action import Action
import math
import numpy as np

class MDP(object):
    def __init__(self, num_states=1000, num_orientations=10):
        # Build the markov model
        # TODO: Potentially move the map listening node here?
        self.markov_model = MarkovModel(num_states=num_states, num_orientations=num_orientations)
        self.markov_model.make_states()
        self.markov_model.build_roadmap()
        self.num_states = num_states

        self.goal_state_idx = 0
        self.reward_radius = 0
        self.rewards = np.empty(self.num_states)

        self.policy = [] # List of actions to take corresponding to each state idx in markov_model.model_states
        self.value_function = [] # List of rewards corresponding to being each state based on the policy

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
        high_reward = 100
        low_reward = -1

        goal_state = self.markov_model.model_states[self.goal_state_idx]
        for state_idx in range(self.num_states):
            s = self.markov_model.model_states[state_idx]
            if goal_state.distance(s) < self.reward_radius:
                self.rewards[state_idx] = high_reward
            else:
                self.rewards[state_idx] = low_reward

    def make_policy(self, state=None):
        if(state != None):
            self.set_goal_state(state)

        # Generate reward vector
        self.set_rewards()

        # Generate a starting, random policy from all available actions
        self.policy = self.get_random_policy()

        change = -1
        while change != 0:
            # Build P matrix.
            p_matrix = self.build_p_matrix()
            # print(p_matrix)

            # Find V of this policy.
            solved = self.solve_value_function()
            if(solved):
                # Calculate new policy.
                change = self.get_new_policy()
                print(change)
            else:
                # Singular matrix solution to V
                self.policy =self.get_random_policy()
        print("Converged!")
        print(self.policy)

    def get_random_policy(self):
        return np.random.choice(Action.get_all_actions(), self.num_states, replace=True)

    def get_new_policy(self):
        gamma = 0.5
        all_actions = Action.get_all_actions()
        total_change = 0
        for state_idx in range(self.num_states):
            ps_matrix = self.build_ps_matrix(state_idx)
            state_rewards = self.rewards[state_idx] + gamma * ps_matrix.dot(self.value_function)
            idx_action = state_rewards.argmax()
            total_change += self.policy[state_idx] - all_actions[idx_action]
            self.policy[state_idx] = all_actions[idx_action]
        return total_change

    def solve_value_function(self):
        I = np.identity(self.num_states)
        gamma = 0.5
        if(np.linalg.det(I - gamma * self.policy) == 0):
            return False
        self.value_function = np.linalg.inv(I - gamma * self.policy).dot(self.rewards)
        print(self.value_function)
        return True

    def build_p_matrix(self):
        p_matrix = np.empty([self.num_states, self.num_states])
        for start_state_idx in range(self.num_states):
            for end_state_idx in range(self.num_states):
                p_matrix[start_state_idx][end_state_idx] = \
                        self.markov_model.get_probability(start_state_idx, end_state_idx, self.policy[start_state_idx])
        return p_matrix

    def build_ps_matrix(self, state_idx):
        all_actions = Action.get_all_actions()
        ps_matrix = np.empty([len(all_actions), self.num_states])
        for i in range(len(all_actions)):
            a = all_actions[i]
            for j in range(self.num_states):
                start_state_idx = j
                ps_matrix[i][j] = self.markov_model.get_probability(start_state_idx, state_idx, a)
        return ps_matrix





if __name__ == "__main__":
    mdp = MDP(num_states=5, num_orientations=1)
    print("model.map.info: {}".format(mdp.markov_model.map.info))
    print("Validate is_collision_free - should be False: {}".format(mdp.markov_model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(mdp.markov_model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    mdp.markov_model.print_states()
    # print(mdp.markov_model.get_probability(3, 0, Action.FORWARD))
    mdp.set_goal_state(State(x=1, y=1, theta=math.radians(40)))
    mdp.set_rewards()
    print(mdp.rewards)
    mdp.make_policy()
