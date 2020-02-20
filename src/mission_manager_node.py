#!/usr/bin/python3

"""
Purpose: Serve as over-arching state machine for the mission
State Machine Definition

Suspended: All systems inactive
STATE 0: Waiting_For_Deployment: Boat is in helicopter, awaiting deployment
TRANSITION CRITERIA: imu-based gravity vector decreases (freefall)
STATE 1: - Deploying: Boat is in air
TRANSITION CRITERIA: imu-based gravity vector re-appears
STATE 2: Landed: Boat is in water
TRANSITION CRITERIA: Orientation of boat known
STATE 3: Self-Righting: Boat knows orientation, correcting antennas
TRANSITION CRITERIA: GPS goal received
STATE 4: GPS-Navigation: Boat is Navigating towards GPS waypoint
TRANSITION CRITERIA: Swept-Tone Frequency Found
STATE 5: Beacon-Navigation: Boat is Navigating towards Beacon
TRANSITION CRITERIA: Visual Confirmation of person
STATE 6: Visual-Navigation: Boat is Visual-Servoing based on the human
TRANSITION CRITERIA: Within max safe distance of human
STATE 7: Buoy: Found the person, aiding payload access + floatation
TRANSITION CRITERIA: Joy Flag enabled on launch
STATE 8: Joystick: Take the boat for a joy ride...get it
"""

import rospy

from std_msgs.msg import Float32MultiArray, Float32, Bool
from geometry_msgs.msg import TwistStamped, PoseStamped


class MissionManagerNode:
    def __init__(self):
        # subscribe to human detections
        self.person_found_sub = rospy.Subscriber("/visual_tracking/person_found", Bool,
                                                 self.person_found_cb, queue_size=1)

        # subscribe to swept tone detection
        self.tone_found_sub = rospy.Subscriber("/kerberos/tone_found", Bool, self.tone_found_cb,
                                               queue_size=1)

        # subscribe to gps_goal
        self.gps_goal_sub = rospy.Subscriber("/gps_translator/goal_pose", PoseStamped,
                                               self.gps_received_cb,
                                               queue_size=1)

        # subscribe to the orientation monitor's current status
        self.orientation_sub = rospy.Subscriber("/orientation_monitor/status", Float32,
                                                self.orientation_status_cb, queue_size=1)

        # publish state
        self.state_pub = rospy.Publisher("/mission_manager/state", Float32, queue_size=1)

        # publish thruster commands
        self.cmd_vel_pub = rospy.Publisher("/mission_manager/cmd_vel", TwistStamped, queue_size=1)

        # publish servo commands
        self.servo_cmd_pub = rospy.Publisher("/mission_manager/servo_cmd", PoseStamped,
                                             queue_size=1)

        self.current_state = 0
        self.current_orientation = 0

    '''
    helper method for quickly publishing state method
    '''
    def pub_status(self):
        status_msg = Float32MultiArray()
        status_msg.data = [self.current_state, self.current_orientation]
        self.state_pub.publish(status_msg)

    '''
    callback for orientation status. this has a number of the state transitions in it, including 
    the recovery behavior
    '''
    def orientation_status_cb(self, float_msg):
        # TRANSITION CRITERIA: 0 --> 1
        if float_msg.data == 0 and self.current_state == 0:
            rospy.logwarn("[M_MNGR] Transition 0 --> 1")
            self.current_state = 1
            self.pub_status()
            return

        # TRANSITION CRITERIA: 1 --> 2
        if float_msg.data != 0 and self.current_state == 1:
            rospy.logwarn("[M_MNGR] Transition 1 --> 2")
            self.current_state = 2
            self.pub_status()
            return

        # TRANSITION CRITERIA 2 --> 3
        if self.current_state == 2:

            self.current_state = 3
            rospy.logwarn("[M_MNGR] Transition 2 --> 3")

            self.current_orientation = float_msg.data

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "body"

            if float_msg.data > 0:
                pose_msg.pose.position.z = 1.0
            elif float_msg.data < 0:
                pose_msg.pose.position.z = -1.0

            self.servo_cmd_pub.publish(pose_msg)

            self.pub_status()
            return

        # TRANSITION CRITERIA N --> 3
        # this is a recovery behavior, if the boat is in some navigation mode, and orientation
        # changes, we need to recover
        if float_msg.data != self.current_orientation and self.current_state > 3:
            self.current_state = 3
            rospy.logwarn("[M_MNGR] Transition {} --> 3".format(self.current_state))

            self.current_orientation = float_msg.data

            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "body"

            if float_msg.data > 0:
                pose_msg.pose.position.z = 1.0
            elif float_msg.data < 0:
                pose_msg.pose.position.z = -1.0

            self.servo_cmd_pub.publish(pose_msg)

            self.pub_status()
            return

    '''
    callback for if a person is found
    '''
    def person_found_cb(self, bool_msg):
        # TRANSITION CRITERIA N > 2 --> 6
        if 2 < self.current_state < 6 and bool_msg.data:
            self.current_state = 6
            rospy.logwarn("[M_MNGR] Transition {} --> 6".format(self.current_state))
            self.pub_status()
            return
    '''
    callback for if tone found 
    '''
    def tone_found_cb(self, bool_msg):
        # TRANSITION CRITERIA N > 2 --> 5
        if 2 < self.current_state < 5 and bool_msg.data:
            self.current_state = 5
            rospy.logwarn("[M_MNGR] Transition {} --> 5".format(self.current_state))
            self.pub_status()
            return
    '''
    callback for if gps received 
    '''
    def gps_received_cb(self, pose_msg):
        # TRANSITION CRITERIA N > 2 --> 5
        if 2 < self.current_state < 4:
            self.current_state = 4
            rospy.logwarn("[M_MNGR] Transition {} --> 4".format(self.current_state))
            self.pub_status()
            return


if __name__ == "__main__":
    rospy.init_node("joy_to_pwm", anonymous=True, disable_signals=False)
    jp = MissionManagerNode()
    rospy.spin()
