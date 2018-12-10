# Simulate a policy and see if it converges.
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from mdptoolbox_test import MDP
from State import State
from Action import Action
import numpy as np
import math

class Robot(object):
    def __init__(self, mdp, goal_state=State(x=0, y=0, theta=0), start_state_idx=1):
        self.mdp = mdp
        # Generate policy
        self.policy, iter, time = mdp.get_policy(goal_state)
        self.goal_state = goal_state
        print("Converged: iter: {}, time: {}".format(iter, time))

        self.kd_tree = self.mdp.markov_model.kd_tree
        self.model_states = self.mdp.markov_model.model_states

        self.robot_state_pub = rospy.Publisher('/robot_state_marker', Marker, queue_size=10)
        self.robot_state_pose_pub = rospy.Publisher('/robot_state_pose', PoseArray, queue_size=10)
        self.state_idx = start_state_idx

    def simulate_policy(self, iterations=50):
        robot_states = self.get_states(iterations=iterations)
        r = rospy.Rate(1)
        i = 0
        num_consec = 0
        prev_state = State(x=0, y=0, theta=0)
        while i < iterations:
            self.mdp.visualize_policy(self.policy, self.goal_state)
            curr_state = robot_states[i]
            print(curr_state)
            print(prev_state)
            if(prev_state == curr_state):
                num_consec += 1
                if(num_consec > 2):
                    break
            else:
                num_consec = 0
            self.robot_state_pub.publish(curr_state.get_marker(r=0.0, g=0.0, b=1.0, scale=0.15))
            robot_pose = PoseArray()
            robot_pose.header.frame_id = "map"
            robot_pose.poses = [curr_state.get_pose()]
            self.robot_state_pose_pub.publish(robot_pose)
            prev_state = curr_state
            i += 1
            r.sleep()
        print("Finished simulation\n")
        print("Would you like to run again?")
        if(raw_input() in ['n', 'NO', 'N', 'no']):
            print("Exiting")
        else:
            print("Enter start state: ")
            self.state_idx = input()
            self.simulate_policy()

    def get_states(self, iterations=100):
        states = []
        for i in range(iterations):
            # Execute the action at the given state.
            action = self.policy[self.state_idx]
            print(Action.to_str(action))
            new_state = self.move(action)
            # Update state idx.
            self.state_idx = self.get_closest_state_idx(new_state, self.state_idx)
            states.append(self.model_states[self.state_idx])
        return states

    def move(self, action):
        # Standard deviations
        # TODO: FIX STANDARD DEVIATIONS. They are not reasonable. However, if they are too small,
        #       then more states need to be generated to have meaningful transitions (not to the state itself)
        pos_sd = 0.01
        theta_sd = 0.01

        start_state = self.model_states[self.state_idx]
        start_x = start_state.x
        start_y = start_state.y
        start_theta = start_state.theta

        linear, angular = Action.get_pose_change(action)

        end_x = start_x + np.random.normal(linear * math.cos(start_theta), pos_sd)
        end_y = start_y + np.random.normal(linear * math.sin(start_theta), pos_sd)
        end_theta = np.random.normal(start_theta + angular, theta_sd) % (2 * math.pi)

        return State(x=end_x, y=end_y, theta=end_theta)

    def get_closest_state_idx(self, sample_state, start_state_idx):
        distance, closest_state_idx = self.kd_tree.query(np.array([sample_state.get_pose_xytheta()]), k=2)
        if(closest_state_idx[0][0] == start_state_idx):
            return np.asscalar(closest_state_idx[0][1])
        return np.asscalar(closest_state_idx[0][0])

if __name__ == "__main__":
    mdp = MDP(num_positions=100, num_orientations=1, grid_debug=True)
    print("model.map.info: {}".format(mdp.markov_model.map.info))
    print("Validate is_collision_free - should be False: {}".format(mdp.markov_model.is_collision_free((0.97926, 1.4726))))  # Hit wall in ac109_1
    print("Validate is_collision_free - should be True: {}".format(mdp.markov_model.is_collision_free((1.2823, 1.054))))  # free in ac109_1
    # mdp.markov_model.print_states()
    # print(mdp.markov_model.get_probability(3, 0, Action.FORWARD))
    robit = Robot(mdp, goal_state=State(x=0, y=-2, theta=math.radians(10)), start_state_idx=0)
    robit.simulate_policy(iterations=20)
