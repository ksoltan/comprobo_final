#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose, PoseArray, PoseStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
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
    def __init__(self, num_positions=500, num_orientations=10):
        # TODO: Interface with SLAM algorithm's published map
        # Initialize map.
        rospy.init_node("neato_mdp") # May break if markov_model is also subscribed...?
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        # Initialize MDP
        self.mdp = MDP(num_positions=num_positions, num_orientations=num_orientations, map=static_map().map)
        self.state_idx = None # Current state idx is unknown.
        self.curr_odom_pose = Pose()
        self.tf_helper = TFHelper()
        # Velocity publisher
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.set_odom)
        self.goal_state = None
        # Visualize robot
        self.robot_state_pub = rospy.Publisher('/robot_state_marker', Marker, queue_size=10)
        self.robot_state_pose_pub = rospy.Publisher('/robot_state_pose', PoseArray, queue_size=10)
        self.goal_state_pub = rospy.Publisher('/goal_state_marker', Marker, queue_size=10)
        # # pose_listener responds to selection of a new approximate robot location (for instance using rviz)
        #
        self.odom_pose = PoseStamped()
        self.odom_pose.header.stamp = rospy.Time(0)
        self.odom_pose.header.frame_id = 'odom'
        #
        rospy.Subscriber("initialpose",
                         PoseWithCovarianceStamped,
                         self.update_initial_pose)

        rospy.Subscriber("move_base_simple/goal",
                        PoseStamped,
                        self.update_goal_state)

    def set_odom(self, msg):
        self.curr_odom_pose = msg.pose.pose

    def set_goal_idxs(self):
        idxs = []
        for i, reward in enumerate(self.mdp.rewards):
            if reward > 0:
                idxs.append(i)
        return idxs

    def update_initial_pose(self, msg):
        """ Callback function to handle re-initializing the particle filter
            based on a pose estimate.  These pose estimates could be generated
            by another ROS Node or could come from the rviz GUI """
        print("Got an initial pose estimate...?")
        self.tf_helper.fix_map_to_odom_transform(msg.pose.pose,
                                                        msg.header.stamp)
        self.tf_helper.send_last_map_to_odom_transform()
        # Wait for the transform to really get sent. Otherwise, it complains that it does not exist.
        rospy.sleep(2.)
        map_pose = self.tf_helper.tf_listener.transformPose('map', self.odom_pose)
        x, y, theta = self.tf_helper.convert_pose_to_xy_and_theta(map_pose.pose)
        self.state_idx = self.mdp.markov_model.get_closest_state_idx(State(x=x, y=y, theta=theta))

    def update_goal_state(self, msg):
        map_pose = self.tf_helper.tf_listener.transformPose('map', msg)
        x, y, theta = self.tf_helper.convert_pose_to_xy_and_theta(map_pose.pose)
        theta = theta % (2 * math.pi)
        self.goal_state = State(x=x, y=y, theta=theta)
        self.goal_state_pub.publish(self.goal_state.get_marker())

    def execute_action(self, policy):
        # Identify which state you are in
        #TODO: Interface with SLAM algorithm to get predicted position

        action = Action.get_all_actions()[policy[self.state_idx]]
        print("Taking action: ", Action.to_str(action))
        new_state = self.move(action)
        # print(new_state)

        # Update state idx.
        self.state_idx = self.mdp.markov_model.get_closest_state_idx(new_state,
                                                start_state_idx=self.state_idx)

    def move(self, action):
        linear, angular = Action.get_pose_change(action)
        print("linear: {}, angular: {}".format(linear, angular))

        twist_msg = Twist()
        twist_msg.linear.x = 0.25 * linear
        twist_msg.angular.z = 0.3 * angular
        # publish this twist message
        self.cmd_vel_publisher.publish(twist_msg)

        start_odom_x, start_odom_y, start_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(self.curr_odom_pose)
        start_odom_theta = start_odom_theta % (2 * math.pi)
        linear_change, angular_change = self.get_change_in_motion(start_odom_x, start_odom_y, start_odom_theta)

        r = rospy.Rate(1)
        while (action == Action.FORWARD and abs(linear_change) < abs(linear)) or (action != Action.FORWARD and abs(angular_change) < abs(angular)):
            map_pose = self.tf_helper.tf_listener.transformPose('map', self.odom_pose)
            x, y, theta = self.tf_helper.convert_pose_to_xy_and_theta(map_pose.pose)
            theta = theta % (2 * math.pi)
            new_state = State(x=x, y=y, theta=theta)
            self.publish_robot_odom(new_state)
            r.sleep() # Wait a little.
            # Check change again.
            linear_change, angular_change = self.get_change_in_motion(start_odom_x, start_odom_y, start_odom_theta)
            print("Lin change: {} Ang change: {}".format(linear_change, angular_change))

        # Convert odom state to map frame.
        self.odom_pose.pose = self.curr_odom_pose
        # self.odom_pose.header.stamp = rospy.Time.now()
        map_pose = self.tf_helper.tf_listener.transformPose('map', self.odom_pose)
        x, y, theta = self.tf_helper.convert_pose_to_xy_and_theta(map_pose.pose)
        theta = theta % (2 * math.pi)
        new_state = State(x=x, y=y, theta=theta)
        self.publish_robot_odom(new_state)
        print("New state is: ")
        print(new_state)
        return new_state

    def stop(self):
        twist_msg = Twist()
        self.cmd_vel_publisher.publish(twist_msg)

    def publish_robot_odom(self, curr_state):
        self.robot_state_pub.publish(curr_state.get_marker(r=0.0, g=0.0, b=1.0, scale=0.15))

        robot_pose = PoseArray()
        robot_pose.header.frame_id = "map"
        robot_pose.poses = [curr_state.get_pose()]
        self.robot_state_pose_pub.publish(robot_pose)

    def get_change_in_motion(self, start_odom_x, start_odom_y, start_odom_theta):
        new_odom_x, new_odom_y, new_odom_theta = self.tf_helper.convert_pose_to_xy_and_theta(self.curr_odom_pose)
        new_odom_theta = new_odom_theta % (2 * math.pi)
        return (new_odom_x - start_odom_x, self.tf_helper.angle_diff(new_odom_theta, start_odom_theta))

    def run(self):
        # TODO: Parametrize goal state, and be able to dynamically update it?
        # Set an initial goal state.
        # goal_state = State(x=1, y=1, theta=math.radians(40))
        # Solve the MDP
        while self.goal_state == None:
            continue

        policy, iter, time = self.mdp.get_policy(self.goal_state)
        self.mdp.visualize_policy(policy, self.goal_state)

        goal_idxs = self.set_goal_idxs()
        print(goal_idxs)

        r = rospy.Rate(0.5)
        while not rospy.is_shutdown():
            if(self.state_idx == None):
                # print("Current state is unknown. Cannot execute policy.")
                continue
            else:
                # Keep checking whether you are in a different state and should
                # execute a different action.
                if self.state_idx in goal_idxs:
                    self.stop()
                    print("Finished executing policy. Would you like to go again?")
                    if(raw_input() in ['n', 'NO', 'N', 'no']):
                        print("Exiting")
                        break
                    else:
                        print("Enter goal state: ")
                        goal_state_idx = input()
                        self.goal_state = self.mdp.markov_model.model_states[goal_state_idx]
                        # Visualize the goal state as sphere.
                        self.goal_state_pub.publish(self.mdp.markov_model.model_states[goal_state_idx].get_marker())

                        # Solve the MDP
                        policy, iter, time = self.mdp.get_policy(self.goal_state)
                        goal_idxs = self.set_goal_idxs()
                else:
                    self.execute_action(policy)


if __name__ == "__main__":
    neato_mdp = NeatoMDP()
    neato_mdp.run()
