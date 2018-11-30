from markov_model import MarkovModel
from State import State
from Action import Action
import math
import numpy as np

'''
    Class: MDP

    Builds the optimal policy for a given markov model and a specified goal
    to define what action the robot should take in any given state to reach
    the goal.

'''
class MDP(object):
    def __init__(self, num_states=1000, num_orientations=10):
        # Build the markov model
        # TODO: Potentially move the map listening node here?
        self.markov_model = MarkovModel(num_states=num_states, num_orientations=num_orientations)
        self.markov_model.make_states()
        self.markov_model.build_roadmap()
        self.num_states = num_states

        self.goal_state_idx = None
        self.reward_radius = None
        self.rewards = np.empty(self.num_states)

        self.policy = [] # List of actions to take corresponding to each state idx in markov_model.model_states
        self.value_function = [] # List of rewards corresponding to being each state based on the policy


    def set_goal(self, state, reward_radius=0.5):
        self.goal_state_idx = self.markov_model.get_closest_state_idx(state)
        # print(self.goal_state_idx)
        # print(self.markov_model.model_states[self.goal_state_idx])
        self.reward_radius = reward_radius


    '''
        Function: udpate_rewards
        Inputs:

        Generates a vector assigning a low reward to all states not within the
        minimum radius around the goal. The high reward is assigned to all
        states that are within this radius, such as multiple orientations of
        the same state.
        
        Note: We might only want one state with a reward. Otherwise, I think
        the robot might want to hit all of the goal states.
    '''
    def udpate_rewards(self):
        """
        Compute and set self.rewards 
        """
        high_reward = 100
        low_reward = -1

        goal_state = self.markov_model.model_states[self.goal_state_idx]
        for state_idx in range(self.num_states):
            s = self.markov_model.model_states[state_idx]
            if goal_state.distance(s) < self.reward_radius:
                self.rewards[state_idx] = high_reward
            else:
                self.rewards[state_idx] = low_reward


    def make_policy(self, goal_state=None):
        gamma = 0.5 # TODO: Either make class attribute, or push out of this function.

        # Start with a random policy and continuously improve it.
        policy = self.get_random_policy()
        
        change = None
        while change != 0:
            # Build P matrix.
            policy_transition_matrix = self.build_policy_transition_matrix(policy)
            # print(policy_transition_matrix)

            # Find value function of this policy.
            value_function = self.solve_value_function(
                policy_transition_matrix,
                self.rewards,
                gamma)

            if value_function == None:
                policy = self.get_random_policy()
                print("Undefined value function, starting over with a new random policy.")
            else:
                # Improve policy based on new value function
                policy, change = self.get_new_policy(policy, gamma)

        print("Converged!")
        print(policy)
        return policy


    def get_random_policy(self):
        return np.random.choice(Action.get_all_actions(), self.num_states, replace=True)


    def get_new_policy(self, policy, gamma):
        all_actions = Action.get_all_actions()
        total_change = 0

        for state_idx in range(self.num_states):
            state_transition_matrix = self.build_state_transition_matrix(state_idx)
            state_rewards = self.rewards[state_idx] + gamma * state_transition_matrix.dot(self.value_function)
            idx_action = state_rewards.argmax()

            total_change += policy[state_idx] - all_actions[idx_action]
            policy[state_idx] = all_actions[idx_action]

        return policy, total_change


    def solve_value_function(self, policy_transition_matrix, rewards, gamma):
        I = np.identity(self.num_states)

        if(np.linalg.det(I - gamma * policy_transition_matrix) == 0):
            print("det(I - gamma * policy_transition_matrix) == 0, cannot solve for value_function")
            return None

        value_function = np.linalg.inv(I - gamma * policy_transition_matrix).dot(rewards)
        print(value_function)
        return value_function


    def build_policy_transition_matrix(self, policy):
        policy_transition_matrix = np.empty([self.num_states, self.num_states])
        for start_state_idx in range(self.num_states):
            for end_state_idx in range(self.num_states):
                policy_transition_matrix[start_state_idx][end_state_idx] = \
                        self.markov_model.get_probability(start_state_idx, end_state_idx, self.policy[start_state_idx])
        return policy_transition_matrix


    def build_state_transition_matrix(self, state_idx):
        all_actions = Action.get_all_actions()
        state_transition_matrix = np.empty([len(all_actions), self.num_states])
        for i in range(len(all_actions)):
            a = all_actions[i]
            for j in range(self.num_states):
                start_state_idx = j
                state_transition_matrix[i][j] = self.markov_model.get_probability(start_state_idx, state_idx, a)
        return state_transition_matrix


if __name__ == "__main__":
    mdp = MDP(num_states=5, num_orientations=1)
    print("model.map.info: {}".format(mdp.markov_model.map.info))
    print("Validate is_collision_free - should be False: {}".format(mdp.markov_model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(mdp.markov_model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    mdp.markov_model.print_states()
    # print(mdp.markov_model.get_probability(3, 0, Action.FORWARD))
    mdp.set_goal(State(x=1, y=1, theta=math.radians(40)))
    mdp.udpate_rewards()
    print(mdp.rewards)
    new_policy = mdp.make_policy()
    print("new policy:\n{}".format(new_policy))
