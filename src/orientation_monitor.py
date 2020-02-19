#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


pub_status = rospy.Publisher("/marv_status_orientation", Float32, queue_size=1)

def imu_callback(msg):
    float_msg = Float32()
    if msg.linear_acceleration.z < -0.85:
        rospy.logwarn("[ORIENTATION_MONITOR] Boat is Right Side Up.")
        float_msg.data = 0
    elif msg.linear_acceleration.z > 0.85:
        rospy.logwarn("[ORIENTATION_MONITOR] Boat is Up Side Down..")
        float_msg.data = 1

    pub_status.publish(float_msg)


sub_imu = rospy.Subscriber("/imu", Imu, imu_callback, queue_size=10)

rospy.init_node("orientation_monitor", anonymous=True)
rospy.spin()

