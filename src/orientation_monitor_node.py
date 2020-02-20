#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32


class OrientationMonitor:
    def __init__(self):
        self.pub_status = rospy.Publisher("/orientation_monitor/status", Float32, queue_size=1)
        self.sub_imu = rospy.Subscriber("/imu", Imu, self.imu_callback, queue_size=10)

    def imu_callback(self, imu_msg):
        float_msg = Float32()
        if imu_msg.linear_acceleration.z < -0.85:
            rospy.logwarn("[ORIENTATION_MONITOR] Boat is Right Side Up.")
            float_msg.data = -1
        elif imu_msg.linear_acceleration.z > 0.85:
            rospy.logwarn("[ORIENTATION_MONITOR] Boat is Up Side Down.")
            float_msg.data = 1
        elif abs(imu_msg.linear_acceleration.z) < 0.5:
            rospy.logwarn("[ORIENTATION_MONITOR] Boat is in freefall.")
            float_msg.data = 0

        self.pub_status.publish(float_msg)


if __name__ == "__main__":
    rospy.init_node("orientation_monitor", anonymous=True)
    om = OrientationMonitor()
    rospy.spin()
