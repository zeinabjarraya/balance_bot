#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
import time

class IMUFilteredExtractor:
    def __init__(self):
        rospy.init_node('imu_filter_node', anonymous=True)

        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)

        # Complementary filter parameters
        self.alpha = 0.98
        self.last_time = None
        self.pitch = 0.0

        # Flag to check initialization
        self.initialized = False

    def imu_callback(self, msg):
        current_time = time.time()

        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x**2 + acc_z**2))

        if not self.initialized:
            # Initialize pitch from accelerometer
            self.pitch = pitch_acc
            self.last_time = current_time
            self.initialized = True
            rospy.loginfo(f"Initialized pitch from accelerometer: {math.degrees(self.pitch):.2f} degrees")
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        pitch_rate_gyro = msg.angular_velocity.y

        # Complementary filter update
        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        rospy.loginfo_throttle(1, f"Filtered Pitch: {math.degrees(self.pitch):.2f} degrees")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IMUFilteredExtractor()
        node.run()
    except rospy.ROSInterruptException:
        pass
