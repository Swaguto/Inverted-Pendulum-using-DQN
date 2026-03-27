import gymnasium as gym
from gymnasium import spaces
import numpy as np
import serial
import time
import math

class HardwarePendulumEnv(gym.Env):
    """
    Gymnasium Hardware Wrapper for Inverted Pendulum.
    Talks to Arduino via Serial.
    """
    def __init__(self, port='COM3', baudrate=115200):
        super(HardwarePendulumEnv, self).__init__()
        
        # 1. Serial Connection
        try:
            self.ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(2) # Wait for Arduino reset
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            print(f"Failed to connect to Serial: {e}")
            self.ser = None

        # 2. Spaces (Matches CartPole-v1 observation)
        # [cart_pos, cart_vel, cos_theta, sin_theta, theta_dot]
        high = np.array([1.0, 10.0, 1.0, 1.0, 20.0], dtype=np.float32)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)
        
        # Actions: Discrete [Left, Right]
        self.action_space = spaces.Discrete(2)
        
        self.state = np.zeros(5)
        self.last_theta = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if self.ser:
            self.ser.write(b"R\n") # Reset Command
            time.sleep(1)
        return self._get_obs(), {}

    def _get_obs(self):
        # In a real environment, we'd wait for a Serial packet
        if self.ser and self.ser.in_waiting > 0:
            line = self.ser.readline().decode('utf-8').strip()
            if line.startswith("S,"):
                parts = line.split(",")
                # Parsing: S, cart_count, cart_vel, pend_count
                cart_pos = float(parts[1]) / 1000.0 # Scale to meters
                cart_vel = float(parts[2]) / 1000.0
                pend_count = float(parts[3])
                
                # Convert pend_count to radians (assume 2000 pulses per rev)
                theta = (pend_count / 2000.0) * 2 * np.pi
                theta_dot = (theta - self.last_theta) / 0.02 # 50Hz
                self.last_theta = theta
                
                self.state = np.array([
                    cart_pos, cart_vel, 
                    np.cos(theta), np.sin(theta), 
                    theta_dot
                ], dtype=np.float32)

        return self.state

    def step(self, action):
        # 1. Send Action to Hardware
        velocity = 1000.0 if action == 1 else -1000.0
        if self.ser:
            self.ser.write(f"V{velocity}\n".encode())

        # 2. Wait for state update (approx 20ms)
        time.sleep(0.02)
        obs = self._get_obs()
        
        # 3. Reward (Standard Swing-Up reward)
        cos_theta = obs[2]
        reward = float(cos_theta)
        
        # 4. Termination (Cart range limit)
        terminated = bool(abs(obs[0]) > 0.3) # 30cm limit
        truncated = False
        
        return obs, reward, terminated, truncated, {}

    def close(self):
        if self.ser:
            self.ser.close()
