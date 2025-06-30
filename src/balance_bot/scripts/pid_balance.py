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

        # Complementary filter parameter
        self.alpha = rospy.get_param("~alpha", 0.98)
        self.pitch = 0.0
        self.last_time = None

        # Inner pitch PID
        self.kp = rospy.get_param("~kp", 30.0)
        self.ki = rospy.get_param("~ki", 1.0)
        self.kd = rospy.get_param("~kd", 7.0)

        # Outer position loop
        self.kp_pos = rospy.get_param("~kp_pos", 0.3)   # reduced
        self.kd_pos = rospy.get_param("~kd_pos", 0.2)
        self.ki_pos = rospy.get_param("~ki_pos", 0.05)  # small integral gain for position
        self.integral_pos = 0.0
        self.max_desired_pitch = math.radians(15)  # limit desired pitch to 15 degrees

        self.integral = 0.0
        self.last_error = 0.0

        self.last_pos = 0.0

        # Target position to hold
        self.target_pos = 0.0

        self.max_effort = rospy.get_param("~max_effort", 20.0)

        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.left_pub = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=10)

        self.current_pos = 0.0
        self.current_vel = 0.0

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def odom_callback(self, msg):
        pos = msg.pose.pose.position.x
        current_time = time.time()
        dt = current_time - self.last_time if self.last_time else 0.0
        vel = (pos - self.last_pos) / dt if dt > 0 else 0.0

        self.current_pos = pos
        self.current_vel = vel

        self.last_pos = pos

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

        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2))
        pitch_pred = self.pitch + pitch_rate_gyro * dt
        self.pitch = self.alpha * pitch_pred + (1 - self.alpha) * pitch_acc
        self.pitch = self.normalize_angle(self.pitch)

        # =====================
        # OUTER POSITION LOOP
        # =====================
         # Position control (PID) with integral term
        pos_error = self.target_pos - self.current_pos
        if abs(pos_error) < 0.01:
            pos_error = 0.0

        vel_error = -self.current_vel
        # Clamp velocity error for stability
        vel_error = max(min(vel_error, 1.0), -1.0)

        # Integrate position error with anti-windup clamp
        self.integral_pos += pos_error * dt
        self.integral_pos = max(min(self.integral_pos, 5.0), -5.0)

        # Desired pitch from position PID
        desired_pitch = (self.kp_pos * pos_error) + (self.kd_pos * vel_error) + (self.ki_pos * self.integral_pos)
        desired_pitch = max(min(desired_pitch, self.max_desired_pitch), -self.max_desired_pitch)
        # =====================
        # INNER PITCH PID
        # =====================
        error = desired_pitch - self.pitch
        self.integral += error * dt
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error

        effort = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Limit effort to motor capability
        effort = max(min(effort, self.max_effort), -self.max_effort)

        # ================
        # FAIL-SAFE
        # ================
        if abs(math.degrees(self.pitch)) > 45:
            self.integral = 0.0
            self.left_pub.publish(0.0)
            self.right_pub.publish(0.0)
            rospy.logwarn("Pitch out of safe range! Motors stopped.")
            return

        self.left_pub.publish(effort)
        self.right_pub.publish(effort)

        rospy.loginfo_throttle(1, f"Pitch: {math.degrees(self.pitch):.2f}°, DesiredPitch: {math.degrees(desired_pitch):.2f}°, Effort: {effort:.2f}, X: {self.current_pos:.3f}, Vel: {self.current_vel:.3f}")

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        CascadePIDBalanceBot().run()
    except rospy.ROSInterruptException:
        pass
