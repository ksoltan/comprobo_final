'''
    Code to visualize markov_model and MDP generation, step by step.
'''
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.patches import Circle
from matplotlib.collections import PatchCollection
import numpy as np
from markov_model import MarkovModel
from Action import Action

'''
    Split a probability matrix by action.
    Expects a (S, S, A) matrix.
'''
def scatter_3D_prob_matrix(matrix):
    fig, axes = plt.subplots(matrix.shape[2], 1)
    plt.subplots_adjust(hspace=1)
    all_actions = Action.get_all_actions()
    for i in range(matrix.shape[2]):
        axes[i].set_title(Action.to_str(all_actions[i]))
        if(matrix.shape[0] > 20):
            viz_2D_pcolor_matrix(matrix[:, :, i], axes[i])
        else:
            viz_2D_scatter_matrix(matrix[:, :, i], axes[i])

def viz_2D_pcolor_matrix(matrix, ax):
    ax.set_xlabel('Start State')
    ax.set_ylabel('End State')
    ax.pcolor(matrix, edgecolors='k', linewidths=0.5)

def viz_2D_scatter_matrix(matrix, ax):
    patches = []
    colors  = []
    ax.set_xlabel('Start State')
    ax.set_ylabel('End State')
    for x in range(matrix.shape[0]):
        for y in range(matrix.shape[1]):
            prob = matrix[x][y]
            ax.scatter(x, y, c=np.clip([1 - prob, 0, prob], 0, 1), s=50, alpha=.4)


if __name__ == "__main__":
    model = MarkovModel(num_positions=100, num_orientations=1)
    print("model.map.info: {}".format(model.map.info))
    model.make_states()
    print("Validate is_collision_free - should be False: {}".format(model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    model.build_roadmap()

    scatter_3D_prob_matrix(model.roadmap)
    plt.show()
