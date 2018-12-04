import numpy as np
import math
from sklearn.neighbors import KDTree, BallTree
from sklearn.neighbors.dist_metrics import DistanceMetric

from State import State

def make_states(num_states=10, num_orientations=1):
    model_states = []
    poses = []
    np.random.seed(0)
    map_x_range = (-3, 3)
    map_y_range = (-2, 2)
    # x, y, z = np.mgrid[0:5, 2:8, -3:3]
    # return np.array(zip(x.ravel(), y.ravel(), z.ravel()))
    while len(model_states) < num_states:
        sampled_position = (np.random.uniform(*map_x_range), np.random.uniform(*map_y_range))

        for i in range(num_orientations):
            angle = np.random.uniform(0, 2 * math.pi)
            model_states.append(State(x=sampled_position[0],
                                           y=sampled_position[1],
                                           theta=angle))
            poses.append([sampled_position[0], sampled_position[1], angle])
    return model_states, np.array(poses)

# def dist_func(a, b):
#     alpha = 1
#     return np.sqrt((a.x - b.x)**2 +
#                      (a.y - b.y)**2 +
#                      alpha*(a.theta - b.theta)**2)

def dist_func(a, b):
    alpha = 1
    return np.sqrt((a[0] - b[0])**2 +
                     (a[1] - b[1])**2 +
                     alpha*(a[2] - b[2])**2)

pyfunc = DistanceMetric.get_metric("pyfunc", func=dist_func)
model_states, X = make_states()
for i in range(X.shape[0]):
    print(X[i, :])
print("TREE TIME")
tree = KDTree(X, leaf_size=4, metric="euclidean")
pts = np.array([(0, 0, 0)])
dist, ind = tree.query(pts, k=1)
for i in ind:
    print(X[i])
    print(np.asscalar(i))
    print(model_states[np.asscalar(i)])

# print(dist)
# print(KDTree.valid_metrics)
# a = np.empty((5, 5, 3))
# b = np.empty((3, 5, 5))
#
# r = 0
# for i in range(a.shape[0]):
#     for j in range(a.shape[1]):
#         for k in range(a.shape[2]):
#             a[i][j][k] = r
#             b[k][i][j] = r
#             r+=0.1
#             print("a[{}][{}][{}] = {} \t b[{}][{}][{}] = {}".format(i, j, k, a[i][j][k], k, i, j, b[k][i][j]))
#
# print(a[0][0][1])
# print(b[1][0][0])
