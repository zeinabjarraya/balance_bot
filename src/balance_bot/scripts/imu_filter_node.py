#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu, JointState
import math
import time

class RobotStateExtractor:
    def __init__(self):
        rospy.init_node('robot_state_node', anonymous=True)

        # IMU filter
        self.alpha = 0.98
        self.pitch = 0.0
        self.last_time = None
        self.initialized = False
        self.pitch_rate = 0.0

        # Wheel velocities
        self.left_vel = 0.0
        self.right_vel = 0.0

        # Subscribers
        rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback)
        rospy.Subscriber('/joint_states', JointState, self.joint_callback)

    def imu_callback(self, msg):
        current_time = time.time()
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        pitch_acc = math.atan2(-acc_y, math.sqrt(acc_x**2 + acc_z**2))
        pitch_rate_gyro = msg.angular_velocity.x

        if not self.initialized:
            self.pitch = pitch_acc
            self.last_time = current_time
            self.initialized = True
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc
        self.pitch_rate = pitch_rate_gyro

    def joint_callback(self, msg):
        try:
            name_idx = {n: i for i, n in enumerate(msg.name)}
            self.left_vel = msg.velocity[name_idx.get('left_wheel_joint', 0)]
            self.right_vel = msg.velocity[name_idx.get('right_wheel_joint', 1)]
        except Exception:
            self.left_vel = 0.0
            self.right_vel = 0.0

    def get_observation(self):
        return [self.pitch, self.pitch_rate, self.left_vel, self.right_vel]

    def run(self):
        rate = rospy.Rate(50)  # 50 Hz
        while not rospy.is_shutdown():
            obs = self.get_observation()
            rospy.loginfo_throttle(1, f"Observation: pitch={math.degrees(obs[0]):.2f}°, pitch_rate={obs[1]:.2f}, left_vel={obs[2]:.2f}, right_vel={obs[3]:.2f}")
            rate.sleep()
    def _send_effort(self, action):
        """
        action: [left_wheel_effort, right_wheel_effort] or normalized values [-1, 1]
        """
        max_effort = 5.0  # adjust based on your robot limits

        left_effort = max(-max_effort, min(max_effort, action[0] * max_effort))
        right_effort = max(-max_effort, min(max_effort, action[1] * max_effort))

        self.left_pub.publish(Float64(left_effort))
        self.right_pub.publish(Float64(right_effort))


if __name__ == "__main__":
    extractor = RobotStateExtractor()
    extractor.run()
