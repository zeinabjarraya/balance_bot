#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import math

# === PID Constants (Start Safe) ===
Kp = 30.0
Ki = 0.0
Kd = 0.0

# === Effort Limits ===
MAX_EFFORT = 10.0  # You can increase after testing

# === Pitch Correction ===
DESIRED_PITCH = 0.0  # degrees
PITCH_OFFSET = 0.0   # use if upright is not 0°

# === Complementary Filter Constant ===
alpha = 0.98  # Trust gyro more

# === Global Variables ===
pitch = 0.0
previous_time = None
integral = 0.0
previous_error = 0.0

# For filtering derivative spikes
DERIVATIVE_CLAMP = 100.0  # Max absolute value for derivative term

# Anti-windup limits for integral
INTEGRAL_MAX = 10.0
INTEGRAL_MIN = -10.0

# Publishers
left_pub = None
right_pub = None

def imu_callback(data):
    global pitch, previous_time, integral, previous_error

    current_time = rospy.Time.now()
    if previous_time is None:
        previous_time = current_time
        return
    dt = (current_time - previous_time).to_sec()
    previous_time = current_time

    # === IMU Data ===
    acc_x = data.linear_acceleration.x
    acc_y = data.linear_acceleration.y
    acc_z = data.linear_acceleration.z

    # === Pitch from Accelerometer (correct axis config: Y forward, Z up) ===
    pitch_acc = math.degrees(math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2)))

    # === Gyro integration (gyro_y in rad/s) ===
    gyro_y = data.angular_velocity.y
    pitch_gyro = pitch + math.degrees(gyro_y * dt)

    # === Complementary Filter ===
    pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc
    pitch += PITCH_OFFSET

    # === PID Calculation ===
    error = DESIRED_PITCH - pitch
    
    # Anti-windup on integral term
    integral += error * dt
    if integral > INTEGRAL_MAX:
        integral = INTEGRAL_MAX
    elif integral < INTEGRAL_MIN:
        integral = INTEGRAL_MIN

    derivative_raw = (error - previous_error) / dt if dt > 0 else 0.0
    
    # Clamp derivative to avoid spikes
    if derivative_raw > DERIVATIVE_CLAMP:
        derivative = DERIVATIVE_CLAMP
    elif derivative_raw < -DERIVATIVE_CLAMP:
        derivative = -DERIVATIVE_CLAMP
    else:
        derivative = derivative_raw

    previous_error = error

    # === Raw Effort ===
    effort = Kp * error + Ki * integral + Kd * derivative

    # === Clip Effort ===
    effort = max(min(effort, MAX_EFFORT), -MAX_EFFORT)

    # === Optional Invert Direction ===
    # effort = -effort  # Uncomment if robot moves wrong direction when tilting

    # === Publish Effort ===
    left_pub.publish(effort)
    right_pub.publish(effort)

    rospy.loginfo(f"Pitch: {pitch:.2f}°, Error: {error:.2f}, Effort: {effort:.2f}, dt: {dt:.2f}")

def pid_balance_node():
    global left_pub, right_pub

    rospy.init_node('pid_balance')
    rospy.Subscriber("/balance_bot/imu_data", Imu, imu_callback)

    left_pub = rospy.Publisher("/left_wheel_effort_controller/command", Float64, queue_size=10)
    right_pub = rospy.Publisher("/right_wheel_effort_controller/command", Float64, queue_size=10)

    rospy.loginfo("✅ PID balance node running with complementary filter and tuned control logic.")
    rospy.spin()

if __name__ == "__main__":
    pid_balance_node()
