#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import time

class PIDBalanceNode:
    def __init__(self):
        rospy.init_node('pid_balance_node', anonymous=True)

        # Subscribers
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)

        # Publishers for wheel effort commands
        self.effort_pub_left = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=10)
        self.effort_pub_right = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=10)

        # PID parameters
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.0

        self.max_effort = 1.0  # Motor effort limits

        # PID state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

        # Complementary filter for pitch
        self.alpha = 0.98
        self.pitch = 0.0

    def imu_callback(self, msg):
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        if dt <= 0:
            return
        self.last_time = current_time

        # Gyro and accelerometer readings
        pitch_rate_gyro = msg.angular_velocity.y
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Calculate pitch angle from accelerometer
        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))

        # Complementary filter to combine gyro and accel
        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        # PID control
        error = 0.0 - self.pitch

        # Deadband for small error to avoid jitter
        if abs(error) < 0.005:  # ~0.3 degrees
            error = 0.0

        self.integral += error * dt

        # Derivative disabled for now to avoid noise issues
        derivative = 0.0

        effort = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Clamp effort to motor limits [-1, 1]
        effort = max(min(effort, self.max_effort), -self.max_effort)

        # Publish the same effort to both wheels
        self.effort_pub_left.publish(Float64(effort))
        self.effort_pub_right.publish(Float64(effort))

        # Log info
        pitch_deg = self.pitch * 180.0 / math.pi
        rospy.loginfo_throttle(1, f"Pitch: {pitch_deg:.2f}° | Effort: {effort:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PIDBalanceNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
