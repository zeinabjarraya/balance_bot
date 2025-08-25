#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class TwoLoopBalanceBot:
    def __init__(self):
        rospy.init_node('two_loop_balance_controller')
        self.motor_sign = rospy.get_param("~motor_sign", -1.0)  # try -1.0 first given your observation

        # === Complementary filter ===
        self.alpha = rospy.get_param("~alpha", 0.98)
        self.pitch = 0.0
        self.last_time = None

        # === Outer loop (angle → target rate) ===
        self.kpa = rospy.get_param("~kpa", 0.0)  # P gain for angle
        self.kia = rospy.get_param("~kia", 0.0)   # I gain for angle
        self.kda = rospy.get_param("~kda", 0.0)   # D gain for angle
        self.ida = 0.0  # angle loop integral
        self.last_angle_error = 0.0
        self.max_angle_i = rospy.get_param("~max_angle_i", 1.0)

        # === Inner loop (rate → motor effort) ===
        self.kpr = rospy.get_param("~kpr", 2.0)   # P gain for rate
        self.kir = rospy.get_param("~kir", 0.0)  # I gain for rate
        self.kdr = rospy.get_param("~kdr", 0.05) # D gain for rate
        self.idr = 0.0  # rate loop integral
        self.last_rate_error = 0.0
        self.max_rate_i = rospy.get_param("~max_rate_i", 0.5)

        # === Limits & filters ===
        self.max_effort = rospy.get_param("~max_effort", 5.0)
        self.deadband = rospy.get_param("~deadband", 0.01)
        self.max_dt = rospy.get_param("~max_dt", 0.02)
        self.max_slew_per_sec = rospy.get_param("~max_slew_per_sec", 20.0)
        self.last_effort = 0.0

        # === Gyro bias calibration ===
        self.gyro_bias = 0.0
        self.calib_samples = rospy.get_param("~calib_samples", 200)
        self._calib_sum = 0.0
        self._calib_count = 0
        self.calibrated = False

        # === ROS setup ===
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        self.left_pub = rospy.Publisher('/left_wheel_effort_controller/command', Float64, queue_size=10)
        self.right_pub = rospy.Publisher('/right_wheel_effort_controller/command', Float64, queue_size=10)

        rospy.loginfo("Two-loop balance controller with derivative started.")

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def imu_callback(self, msg):
        # --- Timing ---
        current_time = msg.header.stamp.to_sec() if msg.header.stamp.to_sec() != 0.0 else rospy.Time.now().to_sec()
        if self.last_time is None:
            self.last_time = current_time
            # collect gyro bias samples
            self._calib_sum += msg.angular_velocity.x
            self._calib_count += 1
            if self._calib_count >= self.calib_samples:
                self.gyro_bias = self._calib_sum / float(self._calib_count)
                self.calibrated = True
                rospy.loginfo(f"Gyro bias calibrated: {self.gyro_bias:.6f} rad/s")
            return

        dt = current_time - self.last_time
        self.last_time = current_time
        if dt <= 0.0:
            return
        if dt > self.max_dt:
            dt = self.max_dt

        # --- Calibration phase ---
        if not self.calibrated:
            self._calib_sum += msg.angular_velocity.x
            self._calib_count += 1
            if self._calib_count >= self.calib_samples:
                self.gyro_bias = self._calib_sum / float(self._calib_count)
                self.calibrated = True
                rospy.loginfo(f"Gyro bias calibrated: {self.gyro_bias:.6f} rad/s")
            return

        # --- IMU data ---
        gyro_y = msg.angular_velocity.x - self.gyro_bias
        acc_x, acc_y, acc_z = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        # --- Complementary filter ---
        acc_norm = math.sqrt(acc_x**2 + acc_y**2 + acc_z**2)
        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))
        G = 9.81
        acc_threshold = rospy.get_param("~acc_threshold", 1.5)
        use_acc = abs(acc_norm - G) < acc_threshold

        pitch_pred = self.pitch + gyro_y * dt
        self.pitch = self.alpha * pitch_pred + (1 - self.alpha) * pitch_acc if use_acc else pitch_pred
        self.pitch = self.normalize_angle(self.pitch)

        # === Outer loop: Angle control ===
        target_pitch = 0.0  # Upright
        angle_error = target_pitch - self.pitch
        self.ida += angle_error * dt
        self.ida = max(min(self.ida, self.max_angle_i), -self.max_angle_i)
        angle_derivative = (angle_error - self.last_angle_error) / dt
        self.last_angle_error = angle_error
        target_rate = -(self.kpa * angle_error + self.kia * self.ida + self.kda * angle_derivative)

        # === Inner loop: Rate control ===
        rate_error = target_rate - gyro_y
        self.idr += rate_error * dt
        self.idr = max(min(self.idr, self.max_rate_i), -self.max_rate_i)
        rate_derivative = (rate_error - self.last_rate_error) / dt
        self.last_rate_error = rate_error
        effort = -(self.kpr * rate_error + self.kir * self.idr + self.kdr * rate_derivative)

        # === Effort conditioning ===
        if abs(effort) < self.deadband:
            effort = 0.0
        effort = max(min(effort, self.max_effort), -self.max_effort)

        # Slew rate limit
        de_max = self.max_slew_per_sec * dt
        delta = effort - self.last_effort
        if delta > de_max:
            effort = self.last_effort + de_max
        elif delta < -de_max:
            effort = self.last_effort - de_max
        self.last_effort = effort

        # === Safety cutoff ===
        if abs(math.degrees(self.pitch)) > 45:
            self.ida = 0.0
            self.idr = 0.0
            self.left_pub.publish(0.0)
            self.right_pub.publish(0.0)
            rospy.logwarn("Pitch out of range. Motors stopped.")
            return

        # === Publish ===
        self.left_pub.publish(effort)
        self.right_pub.publish(effort)
        # replace the publish lines
        #self.left_pub.publish(self.motor_sign * effort)
        #self.right_pub.publish(self.motor_sign * effort)
        rospy.loginfo_throttle(
            0.05,
            f"Pitch: {math.degrees(self.pitch):.2f}°, TargetRate: {target_rate:.2f}, "
            f"RateErr: {rate_error:.2f}, Effort: {effort:.2f}"
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        TwoLoopBalanceBot().run()
    except rospy.ROSInterruptException:
        pass
