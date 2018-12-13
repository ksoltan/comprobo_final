#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.srv import GetMap
from nav_msgs.msg import Odometry
from mdp import MDP
from State import State
from Action import Action
from helper_functions import TFHelper
import tf
import math
'''
    Class: NeatoMDP

    Executes a policy given a static map and some goals.

'''
class NeatoMDP(object):
    def __init__(self, num_positions=100, num_orientations=10):
        # TODO: Interface with SLAM algorithm's published map
        # Initialize map.
        rospy.init_node("neato_mdp") # May break if markov_model is also subscribed...?
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        # Initialize MDP
        self.mdp = MDP(num_positions=num_positions, num_orientations=num_orientations, map=static_map().map)
        self.state_idx = 0 # Current state idx is unknown.
        self.curr_odom_pose = Pose()
        self.tf_helper = TFHelper()
        # Velocity publisher
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.set_odom)
        # # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        #
        # self.base_link_pose = PoseStamped()
        # self.base_link_pose.header.stamp = rospy.Time(0)
        # self.base_link_pose.header.frame_id = 'base_link'
        #
        # rospy.Subscriber("initialpose",
        #                  PoseWithCovarianceStamped,
        #                  self.update_initial_pose)

    def set_odom(self, msg):
        self.curr_odom_pose = msg.pose.pose

    def set_goal_idxs(self):
        idxs = []
        i = 0
        for reward in self.mdp.rewards:
            if reward > 0:
                idxs.append(i)
            i += 1
        return idxs

    # def update_initial_pose(self, msg):
    #     """ Callback function to handle re-initializing the particle filter
    #         based on a pose estimate.  These pose estimates could be generated
    #         by another ROS Node or could come from the rviz GUI """
    #     xy_theta = \
    #         self.tf_helper.convert_pose_to_xy_and_theta(msg.pose.pose)
    #
    #     self.tf_helper.fix_map_to_odom_transform(msg.pose.pose,
    #                                                     msg.header.stamp)
    #     self.tf_helper.send_last_map_to_odom_transform()

    #     new_state = State(x=self.current_pose.linear.x, y=self.current_pose.linear.y, theta=self.current_pose.angular.z)
    #     self.state_idx = self.mdp.markov_model.get_closest_state_idx(new_state, start_state_idx=self.state_idx)

    def execute_action(self, policy):
        # Identify which state you are in
        #TODO: Interface with SLAM algorithm to get predicted position

        action = Action.get_all_actions()[policy[self.state_idx]]
        new_state = self.move(action)
        # print(new_state)

        # Update state idx.
        self.state_idx = self.mdp.markov_model.get_closest_state_idx(new_state,
                                                start_state_idx=self.state_idx)

    def move(self, action):
        linear, angular = Action.get_pose_change(action)

        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular
        # publish this twist message
        self.cmd_vel_publisher.publish(twist_msg)

        start_odom = self.curr_odom_pose
        linear_change, angular_change = self.get_change_in_motion(start_odom)
        r = rospy.Rate(0.5)
        while(linear_change < linear and angular_change < angular):
            r.sleep() # Wait a little.
            # Check change again.
            linear_change, angular_change = self.get_change_in_motion(start_odom)
        new_odom_x, new_odom_y, new_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(self.curr_odom_pose)
        return State(x=new_odom_x, y=new_odom_y, theta=new_odom_theta)

    def get_change_in_motion(self, start_odom):
        start_odom_x, start_odom_y, start_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(start_odom)
        new_odom_x, new_odom_y, new_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(self.curr_odom_pose)
        return (new_odom_x - start_odom_x, self.tf_helper.angle_diff(new_odom_theta, start_odom_theta))

    def run(self):
        # TODO: Parametrize goal state, and be able to dynamically update it?
        # Set an initial goal state.
        goal_state = State(x=1, y=1, theta=math.radians(40))
        # Solve the MDP
        policy, iter, time = self.mdp.get_policy(goal_state)
        goal_idxs = self.set_goal_idxs()
        print(goal_idxs)
        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            if(self.state_idx == -1):
                print("Current state is unknown. Cannot execute policy.")
            else:
                # Keep checking whether you are in a different state and should
                # execute a different action.
                if self.state_idx in goal_idxs:
                    print("Finished executing policy. Would you like to go again?")
                    if(raw_input() in ['n', 'NO', 'N', 'no']):
                        print("Exiting")
                        break
                    else:
                        print("Enter goal state: ")
                        goal_state_idx = input()
                        goal_state = self.mdp.markov_model.model_states[goal_state_idx]
                        # Solve the MDP
                        policy, iter, time = self.mdp.get_policy(goal_state)
                        goal_idxs = self.set_goal_idxs()
                else:
                    self.execute_action(policy)
            r.sleep()

if __name__ == "__main__":
    neato_mdp = NeatoMDP()
    neato_mdp.run()
