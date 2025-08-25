#!/usr/bin/env python3
"""
balance_bot_env.py
A Gymnasium environment that wraps your ROS/Gazebo self-balancing robot.

Topics (as used):
  IMU:        /balance_bot/imu_data         (sensor_msgs/Imu)
  JointStates:/joint_states                 (sensor_msgs/JointState)
  Left Effort:/left_wheel_effort_controller/command   (std_msgs/Float64)
  Right Effort:/right_wheel_effort_controller/command  (std_msgs/Float64)
  Gazebo reset service: /gazebo/set_model_state
"""

import time
import math
import threading
import numpy as np
import gymnasium as gym
from gymnasium import spaces

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu, JointState
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState

DEG2RAD = math.pi / 180.0

class BalanceBotEnv(gym.Env):
    metadata = {"render_modes": []}

    def __init__(self,
                 model_name="balance_bot",
                 max_effort=10.0,
                 dt=0.02,
                 max_episode_steps=1000,
                 pitch_terminate_rad=0.4,
                 imu_topic="/balance_bot/imu_data",
                 joint_topic="/joint_states",
                 left_topic="/left_wheel_effort_controller/command",
                 right_topic="/right_wheel_effort_controller/command"):
        super().__init__()

        # RL / sim parameters
        self.model_name = model_name
        self.max_effort = float(max_effort)
        self.dt = float(dt)                    # control timestep (s)
        self.max_steps = int(max_episode_steps)
        self.pitch_terminate = float(pitch_terminate_rad)

        # ----- ROS init -----
        # Use lazy init if rospy already initialized elsewhere
        if not rospy.core.is_initialized():
            rospy.init_node("balance_bot_gym_env", anonymous=True, disable_signals=True)

        self._lock = threading.Lock()
        self._obs_ready = threading.Event()

        # Publishers (controllers expect Float64)
        self.pub_left = rospy.Publisher(left_topic, Float64, queue_size=1)
        self.pub_right = rospy.Publisher(right_topic, Float64, queue_size=1)

        # State variables
        self.pitch = 0.0         # radians
        self.pitch_rate = 0.0    # rad/s
        self.left_vel = 0.0      # rad/s
        self.right_vel = 0.0     # rad/s

        # Complementary filter internal
        self._alpha = 0.98
        self._last_time = None
        self._initialized = False

        # Subscribers
        rospy.Subscriber(imu_topic, Imu, self._imu_cb, queue_size=1)
        rospy.Subscriber(joint_topic, JointState, self._js_cb, queue_size=1)

        # Gazebo reset service
        try:
            rospy.wait_for_service("/gazebo/set_model_state", timeout=5.0)
            self._set_state_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
        except Exception:
            rospy.logwarn("Gazebo set_model_state service not available; reset will be soft.")
            self._set_state_srv = None

        # Spaces
        # Observation: [pitch (rad), pitch_rate (rad/s), left_vel (rad/s), right_vel (rad/s)]
        obs_high = np.array([np.pi, 50.0, 50.0, 50.0], dtype=np.float32)
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        # Action: scalar in [-1,1] representing symmetric torque (easier) OR shape (2,) for per-wheel
        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)

        # For timing
        self._rate = rospy.Rate(1.0 / self.dt)

        # Internal bookkeeping
        self._step_count = 0
        self._last_obs = np.zeros(4, dtype=np.float32)

        # Wait briefly to get first callbacks
        rospy.sleep(0.1)

    # ---------------- ROS callbacks ----------------
    def _imu_cb(self, msg: Imu):
        """Compute complementary filter-based pitch and store gyro-based pitch_rate."""
        # Use angular_velocity.x as pitch rate (as you discovered)
        gyro_x = msg.angular_velocity.x   # rad/s
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # Compute pitch from accelerometer (radians)
        # Note: sign conventions may vary by IMU mounting; adjust if needed.
        pitch_acc = math.atan2(acc_y, math.sqrt(acc_x * acc_x + acc_z * acc_z))

        now = time.time()
        if not self._initialized:
            # initialize filter state
            with self._lock:
                self.pitch = pitch_acc
                self.pitch_rate = gyro_x
            self._last_time = now
            self._initialized = True
            self._obs_ready.set()
            return

        dt = now - self._last_time if self._last_time is not None else self.dt
        self._last_time = now

        # Complementary filter (pitch in radians)
        with self._lock:
            self.pitch = self._alpha * (self.pitch + gyro_x * dt) + (1.0 - self._alpha) * pitch_acc
            self.pitch_rate = float(gyro_x)
        # mark observation available
        self._obs_ready.set()

    def _js_cb(self, msg: JointState):
        """Extract left/right wheel velocities from joint_states message."""
        try:
            name_idx = {n: i for i, n in enumerate(msg.name)}
            # adapt these joint names to your URDF if different
            lv = msg.velocity[name_idx.get("left_wheel_joint", msg.name[0])]
            rv = msg.velocity[name_idx.get("right_wheel_joint", msg.name[1])] if "right_wheel_joint" in name_idx else (msg.velocity[name_idx.get("wheel_joint", 0)] if msg.velocity else 0.0)
            with self._lock:
                self.left_vel = float(lv)
                self.right_vel = float(rv)
        except Exception:
            # fallback: keep previous values
            pass
        self._obs_ready.set()

    # ---------------- Gym API ----------------
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self._step_count = 0
        self._obs_ready.clear()

        # Stop motors
        self._send_effort(0.0)

        # Soft reset pose in gazebo with a small randomized pitch to avoid trivial policy
        if self._set_state_srv is not None:
            try:
                req = SetModelStateRequest()
                st = ModelState()
                st.model_name = self.model_name
                st.pose.position.x = 0.0
                st.pose.position.y = 0.0
                st.pose.position.z = 0.0
                # small random tilt: ±5 degrees
                init_pitch = np.random.uniform(-5.0 * DEG2RAD, 5.0 * DEG2RAD)
                st.pose.orientation.x, st.pose.orientation.y, st.pose.orientation.z, st.pose.orientation.w = \
                   self._quat_from_rpy(0.0, init_pitch, 0.0)
                # zero velocities
                st.twist.linear.x = 0.0
                st.twist.linear.y = 0.0
                st.twist.linear.z = 0.0
                st.twist.angular.x = 0.0
                st.twist.angular.y = 0.0
                st.twist.angular.z = 0.0
                req.model_state = st
                self._set_state_srv(req)
                
                if not resp.success:
                   rospy.logwarn("Failed to reset robot pose: %s", resp.status_message)
            except Exception as e:
                rospy.logwarn("Failed to call set_model_state: %s", e)

        # Optional: Reset Gazebo world for full clean start
        try:
            rospy.wait_for_service('/gazebo/reset_simulation', timeout=1.0)
            reset_world = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_world()
        except Exception:
            # It’s okay if the service isn’t available; continue
            pass

        # Wait a short time for sensor callbacks to populate observations
        if not self._obs_ready.wait(timeout=2.0):
            rospy.logwarn("No sensor data received during reset; proceeding with current values.")

        obs = self._get_obs()
        return obs, {}   # Gymnasium API: return (obs, info)

    def step(self, action):
        self._step_count += 1

        # Clip + scale action
        if isinstance(action, (list, tuple, np.ndarray)):
            a = float(np.clip(action[0], -1.0, 1.0))
        else:
            a = float(np.clip(action, -1.0, 1.0))
        effort = a * self.max_effort
        self._send_effort(effort)

        # Wait one control period
        try:
            self._rate.sleep()
        except rospy.ROSInterruptException:
            pass

        # Collect observation + reward
        obs = self._get_obs()
        reward = self._compute_reward(obs, effort)

        # Check termination conditions
        terminated = self._terminated(obs)
        truncated = (self._step_count >= self.max_steps)

        if terminated or truncated:
            self._send_effort(0.0)  # stop motors

        return obs, reward, terminated, truncated, {}

    # ---------------- Helpers ----------------
    def _get_obs(self):
        with self._lock:
            obs = np.array([self.pitch, self.pitch_rate, self.left_vel, self.right_vel], dtype=np.float32)
        self._last_obs = obs
        return obs

    def _compute_reward(self, obs, effort):
        pitch, pitch_rate, lv, rv = obs
        # Alive bonus each step
        r_alive = 1.0
        # tuning constants (easy to change)
        k_angle, k_rate, k_act = 2.0, 0.05, 0.001
        # angle term uses radians squared
        r = r_alive - k_angle * float(pitch)**2 - k_rate * float(pitch_rate)**2 - k_act * (effort / max(1.0, self.max_effort))**2
        return r

    def _terminated(self, obs):
        pitch = float(obs[0])
        return abs(pitch) > self.pitch_terminate

    def _send_effort(self, u):
        """Publish symmetric torque to both wheel controllers. Expects `u` in SI units (N·m or controller-specific)."""
        try:
            msg = Float64(data=float(u))
            self.pub_left.publish(msg)
            self.pub_right.publish(msg)
        except Exception as e:
            rospy.logwarn_throttle(5.0, "Failed to publish effort: %s", e)

    @staticmethod
    def _quat_from_rpy(roll, pitch, yaw):
        cy = math.cos(yaw * 0.5); sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5); sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5); sr = math.sin(roll * 0.5)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        return qx, qy, qz, qw

    def close(self):
        # safety: stop actions
        try:
            self._send_effort(0.0)
        except Exception:
            pass
