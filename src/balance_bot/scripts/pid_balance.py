#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

class SimpleBalanceBot:
    def __init__(self):
        rospy.init_node('simple_balance_controller')

        # Complementary filter parameter
        self.alpha = rospy.get_param("~alpha", 0.98)
        self.pitch = 0.0
        self.last_time = None

        # PID constants
        self.kp = rospy.get_param("~kp", 12.0)
        self.ki = rospy.get_param("~ki", 0.05)
        self.kd = rospy.get_param("~kd", 0.5)
        self.integral = 0.0
        self.last_error = 0.0
        self.filtered_derivative = 0.0

        # PID limits
        self.max_effort = rospy.get_param("~max_effort", 5.0)
        self.max_integral = rospy.get_param("~max_integral", 1.0)
        self.deadband = rospy.get_param("~deadband", 0.05)
        self.max_dt = rospy.get_param("~max_dt", 0.02)  # Limit dt to reduce derivative spikes
        self.max_slew_per_sec = rospy.get_param("~max_slew_per_sec",30.0)  # effort units per second
        self.last_effort = 0.0

        # Calibration for gyro bias
        self.gyro_bias = 0.0
        self.calib_samples = rospy.get_param("~calib_samples", 200)
        self._calib_sum = 0.0
        self._calib_count = 0
        self.calibrated = False

        # ROS setup
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
        # --- use ROS time (works with Gazebo sim_time) ---
        if msg.header.stamp.to_sec() != 0.0:
            current_time = msg.header.stamp.to_sec()
        else:
            current_time = rospy.Time.now().to_sec()

        if self.last_time is None:
            self.last_time = current_time
            # collect gyro bias calibration samples
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

        # wait for calibration
        if not self.calibrated:
            self._calib_sum += msg.angular_velocity.x
            self._calib_count += 1
            if self._calib_count >= self.calib_samples:
                self.gyro_bias = self._calib_sum / float(self._calib_count)
                self.calibrated = True
                rospy.loginfo(f"Gyro bias calibrated: {self.gyro_bias:.6f} rad/s")
            return

        # IMU data (confirm this axis is pitch rate for your URDF orientation)
        gyro_y = msg.angular_velocity.x - self.gyro_bias
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Complementary filter for pitch
        #pitch_acc = math.atan2(acc_y, math.sqrt(acc_x ** 2 + acc_z ** 2))
        #pitch_pred = self.pitch + gyro_y * dt
        #self.pitch = self.alpha * pitch_pred + (1 - self.alpha) * pitch_acc
        #self.pitch = self.normalize_angle(self.pitch)
        # compute accelerometer angle
        acc_norm = math.sqrt(acc_x*acc_x + acc_y*acc_y + acc_z*acc_z)
        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))

# if accel magnitude deviates a lot from gravity, rely on gyro integration only
        G = 9.81
        acc_threshold = rospy.get_param("~acc_threshold", 1.5)  # m/s^2 allowed deviation
        use_acc = abs(acc_norm - G) < acc_threshold

        pitch_pred = self.pitch + gyro_y * dt

        if use_acc:
    # standard complementary
                self.pitch = self.alpha * pitch_pred + (1.0 - self.alpha) * pitch_acc
        else:
    # trust gyro integration only when accel unreliable
                self.pitch = pitch_pred

        self.pitch = self.normalize_angle(self.pitch)



        # PID control
        target_pitch = 0.0  # upright
        error = target_pitch - self.pitch

        # derivative from gyro directly
        derivative = gyro_y
        derivative = max(min(derivative, 1.0), -1.0)  # optional clamping
        self.filtered_derivative = 0.95 * self.filtered_derivative + 0.05 * derivative
        # optionally clamp filtered derivative again
        self.filtered_derivative = max(min(self.filtered_derivative, 1.5), -1.5)
        # Conditional anti-windup for integral
        tentative_integral = self.integral + error * dt
        tentative_effort = self.kp * error + self.ki * tentative_integral + self.kd * self.filtered_derivative
        if abs(tentative_effort) < self.max_effort or (tentative_effort * error) < 0:
            self.integral = max(min(tentative_integral, self.max_integral), -self.max_integral)

        # PID effort
        effort = -(self.kp * error + self.ki * self.integral + self.kd * self.filtered_derivative)

        # Apply deadband
        if abs(effort) < self.deadband:
            effort = 0.0

        # Clamp final effort
        effort = max(min(effort, self.max_effort), -self.max_effort)
        de_max = self.max_slew_per_sec * dt
        delta = effort - self.last_effort
        if delta > de_max:
            effort = self.last_effort + de_max
        elif delta < -de_max:
            effort = self.last_effort - de_max
        self.last_effort = effort
        # Safety cutoff
        if abs(math.degrees(self.pitch)) > 45:
            self.integral = 0.0
            self.left_pub.publish(0.0)
            self.right_pub.publish(0.0)
            rospy.logwarn("Pitch out of range. Motors stopped.")
            return

        self.left_pub.publish(effort)
        self.right_pub.publish(effort)

        rospy.loginfo_throttle(
            0.05,
            f"Pitch: {math.degrees(self.pitch):.2f}°, Effort: {effort:.2f}, Gyro Y: {gyro_y:.2f}, "
            f"P: {self.kp * error:.2f}, D: {self.kd * derivative:.2f}, I: {self.ki * self.integral:.2f}"
        )

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        SimpleBalanceBot().run()
    except rospy.ROSInterruptException:
        pass
