# comprobo_final

Blog posts can be found here: https://github.com/ksoltan/comprobo_final/wiki

## Planning Overview
The following diagram explains how the MDP algorithm produces an optimal policy. It glosses over some of the math that makes this computationally tractable, but it gets the idea across. 

(image)

Our implementation mainly features three files.

### MarkovModel.py
This file features a class, MarkovModel, that builds and handles the roadmap. The gist of what it does is adequately described by the first rwo of the graphic above. It relies on the State and Action classes, which can be found in the similarly named files. The most expensive function is get_transition, as it must search find the nearest neighbor. Since every position has states in multiple orientations, we use a kd-tree to find the nearest positions, then a binary search on the sorted list of states by orientation to find the best fit. 

### mdp.py
This file contains the MDP class. It relies on MarkovModel as it must first build a roadmap. It then produces a policy through a converging iterative process described in the graphic above. The main difference is that it calculates the value function and iterates through the policy using linear algebra. 

### neato_mdp.py
This file uses the MDP class to control a neato. Nothing particularly interesting here, it simply uses the map to determine where the robot is, find the nearest state in the roadmap, then execute the action at that state according to the optimal policy given by MDP. 
