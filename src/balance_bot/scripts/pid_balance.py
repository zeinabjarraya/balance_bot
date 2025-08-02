#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math
import time

class SimpleBalanceBot:
    def __init__(self):
        rospy.init_node('simple_balance_controller')

        # Complementary filter parameter
        self.alpha = rospy.get_param("~alpha", 0.98)
        self.pitch = 0.0
        self.last_time = None

        # PID constants
        self.kp = rospy.get_param("~kp", 1.0)
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 0.8)
        self.integral = 0.0
        self.last_error = 0.0

        self.max_effort = rospy.get_param("~max_effort", 10.0)

        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        self.left_pub = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=10)

        rospy.loginfo("Simple inner-loop PID controller started.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def imu_callback(self, msg):
        current_time = time.time()
        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        gyro_y = msg.angular_velocity.x  # pitch rate
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2))
        pitch_pred = self.pitch + gyro_y * dt
        self.pitch = self.alpha * pitch_pred + (1 - self.alpha) * pitch_acc
        self.pitch = self.normalize_angle(self.pitch)

        # PID control
        target_pitch = 0.0  # upright
        error = target_pitch - self.pitch
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        effort = self.kp * error + self.ki * self.integral + self.kd * derivative
        effort = -effort
        effort = max(min(effort, self.max_effort), -self.max_effort)

        # Safety cutoff
        if abs(math.degrees(self.pitch)) > 45:
            self.integral = 0.0
            self.left_pub.publish(0.0)
            self.right_pub.publish(0.0)
            rospy.logwarn("Pitch out of range. Motors stopped.")
            return

        self.left_pub.publish(effort)
        self.right_pub.publish(effort)

        rospy.loginfo_throttle(1, f"Pitch: {math.degrees(self.pitch):.2f}°, Effort: {effort:.2f}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        SimpleBalanceBot().run()
    except rospy.ROSInterruptException:
        pass
