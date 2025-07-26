#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
import math
import time

class IMUFilteredExtractor:
    def __init__(self):
        rospy.init_node('imu_filter_node', anonymous=True)
        self.imu_sub = rospy.Subscriber('/balance_bot/imu_data', Imu, self.imu_callback, queue_size=200)

        self.alpha = 0.98
        self.last_time = None
        self.pitch = 0.0
        self.latest_msg = None  # add this!

    def imu_callback(self, msg):
        self.latest_msg = msg

    def process_imu(self, msg):
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

        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))

        self.pitch = self.alpha * (self.pitch + pitch_rate_gyro * dt) + (1 - self.alpha) * pitch_acc

        pitch_deg = self.pitch * 180.0 / math.pi
        rospy.loginfo_throttle(1, f"Filtered pitch: {pitch_deg:.2f} degrees")

    def run(self):
        rate = rospy.Rate(200)
        while not rospy.is_shutdown():
            if self.latest_msg:
                self.process_imu(self.latest_msg)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = IMUFilteredExtractor()
        node.run()
    except rospy.ROSInterruptException:
        pass
