import rospy
from geometry_msgs.msg import Twist
from nav_msgs.srv import GetMap
from mdptoolbox_test import MDP
from State import State
from Action import Action

'''
    Class: RobotController

    Interfaces with the SLAM algorithm to update the map and its belief in
    position. Executes commands dictated by the path planning policy.

'''
class RobotController(object):
    def __init__(self):
        # TODO: Interface with SLAM algorithm's published map
        rospy.wait_for_service("static_map")
        static_map = rospy.ServiceProxy("static_map", GetMap)
        self.map = static_map().map

        self.mdp = MDP(num_positions=100, num_orientations=100, map=self.map)

        # Velocity publisher
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10, latch=True)

    def execute_action(self, policy):
        # Identify which state you are in
        #TODO: Interface with SLAM algorithm to get predicted position
        current_state = get_current_pose_estimation()

        state_idx = self.mdp.model_states.get_closest_state_idx(current_state)
        action = Action.get_all_actions()[policy[state_idx]]

        linear, angular = Action.get_pose_change(action)

        twist_msg = Twist()
        twist_msg.linear.x = linear
        twist_msg.angular.z = angular

        # publish this twist message
        self.cmd_vel_publisher.publish(twist_msg)

    def update_model(self):
        # If you get a new map, update the markov model and recalculate the policy

    def run(self):
        #TODO: Parametrize goal state, and be able to dynamically update it?
        goal_state = State(x=1, y=1, theta=math.radians(40))
        policy, iter, time = self.mdp.get_policy(goal_state)

        while not rospy.is_shutdown():
            r = rospy.Rate(0.5)
            # Keep checking whether you are in a different state and should
            # execute a different action.
            self.execute_action(policy)
            r.sleep()
