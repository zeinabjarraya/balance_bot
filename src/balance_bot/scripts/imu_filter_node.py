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
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        pitch_rate_gyro = msg.angular_velocity.y
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x**2 + acc_z**2))
        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        # PID controller on pitch angle (desired pitch = 0)
        error = 0.0 - self.pitch
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        effort = self.pid_p * error + self.pid_i * self.integral + self.pid_d * derivative

        # Publish effort commands (both wheels same effort for balance)
        self.effort_pub_left.publish(effort)
        self.effort_pub_right.publish(effort)

        rospy.loginfo_throttle(1, f"Pitch: {self.pitch:.3f} Effort: {effort:.3f}")
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IMUFilteredExtractor()
        node.run()
    except rospy.ROSInterruptException:
        pass
