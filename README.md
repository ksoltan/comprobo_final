# comprobo_final

Blog posts can be found here: https://github.com/ksoltan/comprobo_final/wiki

## Planning Overview
The following diagram explains how the MDP algorithm produces an optimal policy. It glosses over some of the math that makes this computationally tractable, but it gets the idea across. See http://goldberg.berkeley.edu/pubs/rss-Alterovitz2007_RSS.pdf for "The Stochastic Motion Roadmap: A Sampling Framework for Planning with Markov Motion Uncertainty" paper by Alterovitz, Simeon, and Goldberg for a detailed implementation.

![](https://github.com/ksoltan/comprobo_final/blob/master/imgs/mdp_overview_readme.png)

Our implementation mainly features three files.

### markov_model.py
This file features a class, MarkovModel, that builds and handles the roadmap. The gist of what it does is adequately described by the first row of the graphic above. It relies on the State and Action classes, which can be found in the similarly named files. The most expensive function is get_transition, as it must search and find the nearest neighbor. Since every position has states in multiple orientations, we use a kd-tree to find the nearest positions, then perform a binary search on the sorted list of states by orientation to find the best fit.

### mdp.py
This file contains the MDP class. It relies on markov_model as it must first build a roadmap. It then produces a policy through a converging iterative process described in the graphic above. The policy and value functions are calculated very efficiently using linear algebra. See https://github.com/ksoltan/comprobo_final/wiki/Blog-Post-%233-(Planning)#efficiency for more.

### neato_mdp.py
This file uses the MDP class to control a neato. Nothing particularly interesting here, it simply uses an initial 2D pose estimation and odom updates to determine where the robot is, finds the nearest state in the roadmap, then executes the action at that state according to the optimal policy given by MDP. See https://github.com/ksoltan/comprobo_final/wiki/Blog-Post-%233-(Planning)#robot-implementation for a video and next steps.


## SLAM Overview

![](https://github.com/ksoltan/comprobo_final/blob/master/Scans/Poster-%20Mapping.png)

Our implementation of this algorithm can be found in two files.

### gmapping.py
Containing our original MVP, we've built everything else upon this file. Our initial plan behind this project was to first implement GMapping as our MVP, then successively remove and replace the pre-built packages with our own implementations. In this way, we would always have a viable project, and could learn about various parts of the algorithm in great detail. See https://github.com/ksoltan/comprobo_final/wiki/Blog-Post-%231-(SLAM) for more information.

### slam.py
Within this file lives our current implementation of the SLAM algorithm. We first began by diving deep into the Rao-Blackwellized particle filter utilized by GMapping. After this research, we began our implementation of various pieces of SLAM with the Rao-Blackwellized particle filter as a far stretch goal. The current final build of this project is the step just prior to this type of particle filter, with localization and mapping relying on accurate sensor data to create realistic maps of different environments. See https://github.com/ksoltan/comprobo_final/wiki/Blog-Post-%232-(SLAM) for our progress in this file a little more than halfway through the project. A discussion of the final build can be found here https://github.com/ksoltan/comprobo_final/wiki/Blog-Post-%233-(SLAM).
