#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
import time

class IMUFilteredExtractor:
    def __init__(self):
        rospy.init_node('imu_filter_node', anonymous=True)

        # Subscribe to IMU topic
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)

        # Complementary filter parameters
        self.alpha = 0.98  # Adjust if needed (98% gyro, 2% accel)
        self.last_time = None
        self.pitch = 0.0  # Filtered pitch angle

    def imu_callback(self, msg):
        # Time delta
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Get gyro pitch rate (rad/s)
        pitch_rate_gyro = msg.angular_velocity.y

        # Get linear acceleration
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Compute pitch from accelerometer (rad)
        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x**2 + acc_z**2))

        # Complementary filter: combine gyro + accel
        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        # Log data at 1 Hz
        rospy.loginfo_throttle(1,
            f"Filtered Pitch: {self.pitch:.3f} rad | Pitch Rate: {pitch_rate_gyro:.3f} rad/s | "
            f"Linear Acc: X={acc_x:.3f}, Y={acc_y:.3f}, Z={acc_z:.3f} m/s²"
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IMUFilteredExtractor()
        node.run()
    except rospy.ROSInterruptException:
        pass
