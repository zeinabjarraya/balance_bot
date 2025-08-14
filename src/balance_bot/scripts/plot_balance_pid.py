#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64
import matplotlib.pyplot as plt
import math
from collections import deque
import threading

class LivePlotter:
    def __init__(self):
        rospy.init_node('live_balance_pid', anonymous=True)

        # parameters
        self.imu_topic = rospy.get_param('~imu_topic', '/balance_bot/imu_data')
        self.left_topic = rospy.get_param('~left_topic', '/left_wheel_effort_controller/command')
        self.right_topic = rospy.get_param('~right_topic', '/right_wheel_effort_controller/command')
        self.buffer_seconds = rospy.get_param('~buffer_seconds', 10.0)  # history window (s)
        self.update_hz = rospy.get_param('~update_hz', 10.0)          # plot refresh rate (Hz)

        # thread-safety
        self.lock = threading.Lock()

        # data buffers (store timestamps relative to start_time)
        self.start_time = None
        self.pitch_buf = deque()       # (t_rel, pitch_deg)
        self.left_buf = deque()        # (t_rel, val)
        self.right_buf = deque()       # (t_rel, val)

        # subscriptions
        rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=100)
        rospy.Subscriber(self.left_topic, Float64, self.left_cb, queue_size=100)
        rospy.Subscriber(self.right_topic, Float64, self.right_cb, queue_size=100)

        # prepare figure
        plt.ion()
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.line_pitch, = self.ax1.plot([], [], label='Pitch (deg)')
        self.ax1.axhline(0, linestyle='--', linewidth=0.7)
        self.ax1.set_ylabel('Pitch (°)')
        self.ax1.legend()
        self.line_left, = self.ax2.plot([], [], label='Left Effort')
        self.line_right, = self.ax2.plot([], [], label='Right Effort', linestyle='--')
        self.ax2.set_ylabel('Effort')
        self.ax2.set_xlabel('Time (s)')
        self.ax2.legend()
        plt.tight_layout()

        rospy.loginfo("Live PID plotter initialized. Subscribed to: %s, %s, %s",
                      self.imu_topic, self.left_topic, self.right_topic)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def imu_cb(self, msg):
        # compute pitch from accelerometer (matches your offline script)
        if msg.header.stamp.to_sec() != 0.0:
            t = msg.header.stamp.to_sec()
        else:
            t = rospy.Time.now().to_sec()

        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z
        pitch = math.atan2(acc_y, math.sqrt(acc_x**2 + acc_z**2))
        pitch_deg = math.degrees(self.normalize_angle(pitch))

        with self.lock:
            if self.start_time is None:
                self.start_time = t
            t_rel = t - self.start_time
            self.pitch_buf.append((t_rel, pitch_deg))
            self._trim_buffers(t_rel)

    def left_cb(self, msg):
        t = rospy.Time.now().to_sec()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            t_rel = t - self.start_time
            self.left_buf.append((t_rel, msg.data))
            self._trim_buffers(t_rel)

    def right_cb(self, msg):
        t = rospy.Time.now().to_sec()
        with self.lock:
            if self.start_time is None:
                self.start_time = t
            t_rel = t - self.start_time
            self.right_buf.append((t_rel, msg.data))
            self._trim_buffers(t_rel)

    def _trim_buffers(self, current_t):
        cutoff = current_t - self.buffer_seconds
        # pitch
        while self.pitch_buf and self.pitch_buf[0][0] < cutoff:
            self.pitch_buf.popleft()
        # left
        while self.left_buf and self.left_buf[0][0] < cutoff:
            self.left_buf.popleft()
        # right
        while self.right_buf and self.right_buf[0][0] < cutoff:
            self.right_buf.popleft()

    def update_plot(self):
        with self.lock:
            if not self.start_time:
                return

            times_p = [p[0] for p in self.pitch_buf]
            vals_p = [p[1] for p in self.pitch_buf]

            times_l = [p[0] for p in self.left_buf]
            vals_l = [p[1] for p in self.left_buf]

            times_r = [p[0] for p in self.right_buf]
            vals_r = [p[1] for p in self.right_buf]

        # update lines
        if times_p:
            self.line_pitch.set_data(times_p, vals_p)
            xmin = max(0.0, times_p[-1] - self.buffer_seconds)
            xmax = times_p[-1]
            self.ax1.set_xlim(xmin, xmax)
            ymin = min(vals_p) - 5.0
            ymax = max(vals_p) + 5.0
            self.ax1.set_ylim(ymin, ymax)
        else:
            self.line_pitch.set_data([], [])

        if times_l:
            self.line_left.set_data(times_l, vals_l)
        else:
            self.line_left.set_data([], [])

        if times_r:
            self.line_right.set_data(times_r, vals_r)
        else:
            self.line_right.set_data([], [])

        # effort axis limits (dynamic)
        all_eff = (vals_l + vals_r) if (times_l or times_r) else []
        if all_eff:
            ymin = min(all_eff) - 0.1 * abs(min(all_eff))
            ymax = max(all_eff) + 0.1 * abs(max(all_eff))
            # add tiny margin if min==max
            if ymin == ymax:
                ymin -= 0.5; ymax += 0.5
            self.ax2.set_xlim(xmin, xmax if 'xmax' in locals() else self.buffer_seconds)
            self.ax2.set_ylim(ymin, ymax)
        else:
            self.ax2.set_xlim(0, self.buffer_seconds)
            self.ax2.set_ylim(-1, 1)

        # redraw
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def run(self):
        rate = rospy.Rate(self.update_hz)
        while not rospy.is_shutdown():
            self.update_plot()
            rate.sleep()

if __name__ == "__main__":
    try:
        p = LivePlotter()
        p.run()
    except rospy.ROSInterruptException:
        pass
