#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import time

class BalanceController:
    def __init__(self):
        rospy.init_node('balance_controller_node', anonymous=True)

        # PID gains (tune these)
        self.Kp = 40.0
        self.Ki = 0.0
        self.Kd = 0.0

        # PID state
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None

        # Complementary filter parameters
        self.alpha = 0.98
        self.pitch = 0.0

        # Subscribers & Publishers
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        self.left_effort_pub = rospy.Publisher('/balance_bot/left_shaft_effort_controller/command', Float64, queue_size=1)
        self.right_effort_pub = rospy.Publisher('/balance_bot/right_shaft_effort_controller/command', Float64, queue_size=1)

    def imu_callback(self, msg):
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return
        dt = current_time - self.last_time
        self.last_time = current_time

        # Extract gyro pitch rate (rad/s)
        pitch_rate_gyro = msg.angular_velocity.y

        # Extract accelerometer values
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Calculate pitch angle from accelerometer (rad)
        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x**2 + acc_z**2))

        # Complementary filter for pitch
        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        # PID control on pitch
        self.balance_pid(dt)

    def balance_pid(self, dt):
        # Desired pitch is 0 (upright)
        error = 0.0 - self.pitch
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        # PID output (effort command)
        effort = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

        # Saturate effort to reasonable limits (e.g., -10 to 10)
        effort = max(min(effort, 10), -10)

        # Publish effort command to both wheels (same effort for balancing)
        self.left_effort_pub.publish(Float64(effort))
        self.right_effort_pub.publish(Float64(effort))

        rospy.loginfo_throttle(1, f"Pitch: {self.pitch:.3f} rad | PID effort: {effort:.3f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        controller = BalanceController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
