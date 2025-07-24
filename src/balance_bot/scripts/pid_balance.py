#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import math
import time


class CascadePIDBalanceBot:
    def __init__(self):
        rospy.init_node('cascade_pid_balance_controller')

        # Complementary filter
        self.alpha = rospy.get_param("~alpha", 0.98)

        # Inner Pitch PID gains
        self.kp = rospy.get_param("~kp", 5.0)  # ✅ Start safe: 5.0 not 30!
        self.ki = rospy.get_param("~ki", 0.0)
        self.kd = rospy.get_param("~kd", 1.0)

        # Outer Position Loop gains
        self.kp_pos = rospy.get_param("~kp_pos", 0.1)
        self.kd_pos = rospy.get_param("~kd_pos", 0.05)
        self.ki_pos = rospy.get_param("~ki_pos", 0.0)

        self.max_desired_pitch = math.radians(10)  # ±10 deg

        self.max_effort = rospy.get_param("~max_effort", 2.0)  # ✅ Safe limit

        self.pitch = 0.0
        self.pitch_integral = 0.0
        self.last_pitch_error = 0.0

        self.target_pos = 0.0  # Static target for now
        self.pos_integral = 0.0

        self.current_pos = 0.0
        self.current_vel = 0.0
        self.last_pos = None

        self.last_time = None

        # ROS I/O
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.left_pub = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=1)
        self.right_pub = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=1)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        pos = msg.pose.pose.position.x
        if self.last_pos is None:
            self.last_pos = pos
            self.current_vel = 0.0
            return

        self.current_vel = pos - self.last_pos  # m/s approx, assume fast callback
        self.current_pos = pos
        self.last_pos = pos

    def imu_callback(self, msg):
        now = rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = now
            return

        dt = now - self.last_time
        if dt <= 0.0:
            return

        self.last_time = now

        # Complementary filter
        gyro_x = msg.angular_velocity.y
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        try:
            pitch_acc = math.atan2(-acc_x, math.sqrt(acc_y ** 2 + acc_z ** 2))
        except Exception:
            return

        pitch_pred = self.pitch + gyro_x * dt
        self.pitch = self.alpha * pitch_pred + (1 - self.alpha) * pitch_acc
        self.pitch = self.normalize_angle(self.pitch)

        # === OUTER POSITION CONTROLLER ===
        pos_error = self.target_pos - self.current_pos
        vel_error = -self.current_vel

        desired_pitch = (self.kp_pos * pos_error) + (self.kd_pos * vel_error) + (self.ki_pos * self.pos_integral)
        desired_pitch = max(min(desired_pitch, self.max_desired_pitch), -self.max_desired_pitch)

        self.pos_integral += pos_error * dt

        # === INNER PITCH PID ===
        pitch_error = desired_pitch - self.pitch
        self.pitch_integral += pitch_error * dt
        derivative = (pitch_error - self.last_pitch_error) / dt
        self.last_pitch_error = pitch_error

        effort = self.kp * pitch_error + self.ki * self.pitch_integral + self.kd * derivative
        effort = max(min(effort, self.max_effort), -self.max_effort)

        # === FAILSAFE ===
        if abs(math.degrees(self.pitch)) > 45:
            rospy.logwarn_throttle(1, "Failsafe: Pitch out of range, motors off.")
            self.left_pub.publish(0.0)
            self.right_pub.publish(0.0)
            return

        # === PUBLISH ===
        self.left_pub.publish(effort)
        self.right_pub.publish(effort)

        rospy.loginfo_throttle(1,
            f"Pitch: {math.degrees(self.pitch):.2f}°, "
            f"Desired: {math.degrees(desired_pitch):.2f}°, "
            f"Effort: {effort:.2f}, "
            f"Pos: {self.current_pos:.3f}, Vel: {self.current_vel:.3f}"
            f"Pitch raw (deg): {math.degrees(self.pitch):.2f}, Effort: {effort:.2f}"
        )
        rospy.loginfo_throttle(0.1, f"Pitch (deg): {math.degrees(self.pitch):.2f}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        CascadePIDBalanceBot().run()
    except rospy.ROSInterruptException:
        pass
