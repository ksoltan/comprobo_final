'''
    Code to visualize markov_model and MDP generation, step by step.
'''
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import numpy as np
from markov_model import MarkovModel
from mdp import MDP
from Action import Action
from State import State

'''
    Split a probability matrix by action.
    Expects a (S, S, A) matrix.
'''
def scatter_3D_prob_matrix(matrix):
    fig, axes = plt.subplots(matrix.shape[0], 1)
    plt.subplots_adjust(hspace=1)
    all_actions = Action.get_all_actions()
    for i in range(matrix.shape[0]):
        # axes[i].set_title(Action.to_str(all_actions[i]))
        if(matrix.shape[1] > 0):
            viz_2D_pcolor_matrix(matrix[i, :, :], axes[i], fig, 'Start State', 'End State', Action.to_str(all_actions[i]))
        else:
            viz_2D_scatter_matrix(matrix[i, :, :], axes[i], 'Start State', 'End State', Action.to_str(all_actions[i]))

def viz_2D_pcolor_matrix(matrix, ax, fig, xlabel, ylabel, title, vmin=0, vmax=1):
    print(matrix.shape)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    c = ax.pcolor(matrix, edgecolors='k', linewidths=0.5, vmin=vmin, vmax=vmax)
    fig.colorbar(c, ax=ax)

def viz_2D_scatter_matrix(matrix, ax, xlabel, ylabel, title):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    for x in range(matrix.shape[0]):
        for y in range(matrix.shape[1]):
            prob = matrix[x][y]
            ax.scatter(x, y, c=np.clip([1 - prob, 0, prob], 0, 1), s=50, alpha=.4)

def viz_value_function(vector, ax, fig, xlabel, ylabel, title):
    print(vector)
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    c = ax.pcolor([[i] for i in vector], edgecolors='k', linewidths=0.5)
    fig.colorbar(c, ax=ax)

def viz_iteration_process(mdp):
    # Start by generating rancom policy
    mdp.policy = mdp.get_random_policy()
    mdp.set_goal_state(State(x=0, y=1, theta=0))
    mdp.set_rewards()

    print("Initial random policy")
    print(mdp.policy)
    p_matrix_fig, p_matrix_ax = plt.subplots()
    ps_matrix_fig, ps_matrix_ax = plt.subplots()
    v_func_fig, v_func_ax = plt.subplots()
    iteration_count = 0
    print("Would you like to run the first iter?")
    while raw_input() not in ['n', 'NO', 'N', 'no']:
        # Value Function
        # Show P Matrix
        viz_2D_pcolor_matrix(mdp.build_p_matrix(), p_matrix_ax, p_matrix_fig, 'End State', 'Start State', 'P Matrix')
        # Show Value function
        if(mdp.solve_value_function()):
            viz_value_function(mdp.value_function, v_func_ax, v_func_fig, 'Value', 'State', 'Value Function')
            print("Visualized value function.")
        else:
            print("Failed to solve for a value function, trying a new random policy")
            # Singular matrix solution to V
            mdp.policy = mdp.get_random_policy()
        idx = 0
        ps_matrix = mdp.build_ps_matrix(idx)
        print(ps_matrix)
        viz_2D_pcolor_matrix(ps_matrix, ps_matrix_ax, ps_matrix_fig, 'Next State', 'Action', 'PS Matrix', vmin=ps_matrix.min(), vmax=ps_matrix.max())
        plt.show()
        # print("Finished solving value function. Ready to policy iterate.")
        # # Start policy iteration
        # # Show PS matrix
        # idx = 0
        # gamma = 0.999
        # all_actions = Action.get_all_actions()
        # visualize = True
        # total_change = 0
        # for state_idx in range(mdp.num_states):
        #     print("Visualize PS matrix for state {}?\n".format(state_idx))
        #     if(raw_input() in ['n', 'NO', 'N', 'no']):
        #         visualize = False
        #     ps_matrix = mdp.build_ps_matrix(state_idx)
        #     state_rewards = mdp.rewards[state_idx] + gamma * ps_matrix.dot(mdp.value_function)
        #     idx_action = state_rewards.argmax()
        #     total_change += mdp.policy[state_idx] - all_actions[idx_action]
        #     mdp.policy[state_idx] = all_actions[idx_action]
        #     if(visualize):
        #         viz_2D_pcolor_matrix(mdp.build_ps_matrix(idx), p_matrix_ax, p_matrix_fig, 'Start State', 'End State', 'PS Matrix')
        #     idx += 1
        #     plt.show()
        #
        # print("iteration {}".format(iteration_count))
        # iteration_count += 1
        # # Find V of this policy.
        # solved = mdp.solve_value_function()
        # if(solved):
        #     # Calculate new policy.
        #     mdp.get_new_policy()
        # print("Would you like to run the next iter?")


if __name__ == "__main__":
    mdp = MDP(num_positions=10, num_orientations=1)
    # scatter_3D_prob_matrix(mdp.markov_model.roadmap)

    viz_iteration_process(mdp)

    # fig, ax = plt.subplots()
    # mdp.set_goal_state(State(x=0, y=1, theta=0))
    # # Generate reward vector
    # mdp.set_rewards()
    # viz_value_function(mdp.rewards, ax, fig, 'State', 'Rewards', 'Rewards')

    plt.show()
