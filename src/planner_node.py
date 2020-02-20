#!/usr/bin/python3

import rospy
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Pose
import numpy as np


class MARVPlanner:
    def __init__(self):
        # subscribe to gps goal
        self.gps_goal_sub = rospy.Subscriber("/gps_translator/goal", PoseStamped,
                                             self.gps_goal_callback, queue_size=1)
        # subscribe to visual goal
        self.visual_goal_sub = rospy.Subscriber("/visual_tracking/goal", PoseStamped,
                                                self.gps_goal_callback, queue_size=1)

        # subscribe to keberos goal
        self.kerberos_goal_sub = rospy.Subscriber("/kerberos/goal", PoseStamped,
                                                  self.gps_goal_callback, queue_size=1)

        # subscribe to state
        self.state_sub = rospy.Subscriber("/mission_manager/state", Float32MultiArray,
                                          self.state_callback, queue_size=1)

        # subscribe to the pose
        self.pose_sub = rospy.Subscriber("/state_estimation/current_pose", PoseStamped,
                                         self.pose_callback, queue_size=1)

        # publish thruster command
        self.cmd_vel_pub = rospy.Publisher("/thruster_command/cmd_vel", TwistStamped, queue_size=1)

        self.current_state = 0
        self.current_goal = [0, 0, 0]
        self.current_pose = Pose()

    '''
    Run the whole loop!
    '''
    def run(self):
        if 3 < self.current_state < 7:
            rospy.logwarn("[MPlanner] Running Loop")


    '''
    callback to properly process gps based goals
    '''
    def gps_goal_callback(self, pose_msg):
        if self.current_state == 4:
            q = pose_msg.pose.orientation

            # quaternion maths! in order to convert from q to euler yaw
            yaw = np.arctan2(2.0 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            self.current_goal[0] = pose_msg.pose.position.x
            self.current_goal[1] = pose_msg.pose.position.y
            self.current_goal[2] = yaw

    '''
    callback to properly process keberos based goals
    '''
    def keberos_goal_callback(self, pose_msg):
        if self.current_state == 4:
            q = pose_msg.pose.orientation

            # quaternion maths! in order to convert from q to euler yaw
            yaw = np.arctan2(2.0 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            self.current_goal[0] = pose_msg.pose.position.x
            self.current_goal[1] = pose_msg.pose.position.y
            self.current_goal[2] = yaw

    '''
    callback to properly process visual based goals
    '''
    def visual_goal_callback(self, pose_msg):
        if self.current_state == 4:
            q = pose_msg.pose.orientation

            # quaternion maths! in order to convert from q to euler yaw
            yaw = np.arctan2(2.0 * (q.z * q.w + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

            self.current_goal[0] = pose_msg.pose.position.x
            self.current_goal[1] = pose_msg.pose.position.y
            self.current_goal[2] = yaw

    '''
    callback for current state, also serves as the "keyed" callback that calls the run method
    '''
    def state_callback(self, float_arr_msg):
        self.current_state = float_arr_msg.data[0]
        self.run()

    '''
    callback for getting current position
    '''
    def pose_callback(self, pose_msg):
        self.current_pose = pose_msg.pose

    '''
    method for calculating current desired control based on state and goal
    '''
    def calculate_control(self):
        pass


if __name__ == "__main__":
    rospy.init_node("planner_node", anonymous=True)
    mp = MARVPlanner()
    rospy.spin()
