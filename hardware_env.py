import gymnasium as gym
import numpy as np
import time
import serial

class HardwarePendulumEnv(gym.Env):
    """
    Hardware wrapper for the inverted pendulum. 
    Matches the state/action spaces of the simulation environment but routes
    commands through PySerial to the Arduino.
    """
    def __init__(self, port='COM3', baudrate=250000):
        super(HardwarePendulumEnv, self).__init__()
        
        # Max stepper velocity (Steps per second). Increased with ramp support.
        self.max_velocity = 35000 
        
        # Action space is discrete (0: Left, 1: Right)
        self.action_space = gym.spaces.Discrete(2)
        
        # State space matches train_dqn.py: [cart_pos (unnormalized), cart_vel, pole_angle (-pi to pi), pole_vel]
        self.observation_space = gym.spaces.Box(
            low=np.array([-1.0, -10.0, -np.pi, -20.0]),
            high=np.array([1.0, 10.0, np.pi, 20.0]),
            dtype=np.float32
        )
        
        self.cart_limit = 0.3 # meters
        
        # Hardware calibration constants
        # LPD3806 600PPR produces 2400 edges per revolution via PCINT
        self.PEND_COUNTS_PER_REV = 2400.0
        
        # If cart encoder tracks a 40mm circumference pulley (e.g. 20T GT2)
        # 1 rev = 2400 counts = 0.04 meters
        # CART_COUNTS_PER_METER = 2400 / 0.04 = 60000
        # Adjust this parameter if your mechanical setup is different!
        self.CART_COUNTS_PER_METER = 60000.0 
        
        # State tracking to compute velocities
        self.last_cart_pos = 0.0
        self.last_theta = 0.0
        self.last_time = time.time()
        
        print(f"Connecting to Arduino on {port}...")
        self.ser = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2) # Wait for Arduino to reset upon connection
        
        # Flush buffers
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        
    def _get_obs(self):
        # 1. Ask Arduino for state
        self.ser.write(b"S\n")
        
        line = self.ser.readline().decode('utf-8').strip()
        cart_count = 0
        pend_count = 0
        
        try:
            if ',' in line:
                cart_count_str, pend_count_str = line.split(',')
                cart_count = int(cart_count_str)
                pend_count = int(pend_count_str)
        except ValueError:
            print(f"Warning: Corrupted serial line read: {line}")
            
        # 2. Convert to Physics Units
        # User confirmed Pendulum Right = Negative. We need Right = Positive.
        theta = -(pend_count / self.PEND_COUNTS_PER_REV) * 2 * np.pi 
        cart_pos = cart_count / self.CART_COUNTS_PER_METER
        
        # 3. Compute Velocities via discrete derivative
        current_time = time.time()
        dt = current_time - self.last_time
        if dt < 0.001: 
            dt = 0.001 # prevent div by zero
            
        cart_vel = (cart_pos - self.last_cart_pos) / dt
        
        # Shortest angular distance to handle wrapping
        d_theta = (theta - self.last_theta)
        d_theta = (d_theta + np.pi) % (2 * np.pi) - np.pi
        pole_vel = d_theta / dt
        
        self.last_cart_pos = cart_pos
        self.last_theta = theta
        self.last_time = current_time
        
        # 4. Normalize appropriately as neural net expects
        # Model was trained with 0 as UPRIGHT.
        # If theta=0 is bottom, then upright is theta=pi.
        # Internal model angle = theta - pi (wrapped)
        model_theta = (theta - np.pi + np.pi) % (2 * np.pi) - np.pi # matches wrap logic
        # wait, (theta - pi) wrapped is what we want.
        internal_angle = (theta - np.pi + np.pi) % (2 * np.pi) - np.pi
        
        obs = np.array([
            cart_pos,           # raw cart position in meters
            cart_vel,           # raw cart velocity in m/s
            internal_angle,     # wrapped pole angle (0 is upright)
            pole_vel            # raw pole angular velocity
        ], dtype=np.float32)
        
        return obs
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # We can't teleport hardware. 
        # Stop motor and assume the user centers everything manually
        self.ser.write(b"A0\n")
        time.sleep(0.1)
        self.ser.reset_input_buffer()
        
        self.last_time = time.time()
        self.last_cart_pos = 0.0
        self.last_theta = 0.0
        
        return self._get_obs(), {}
        
    def step(self, action):
        # Action is Discrete (0: FastL, 1: SlowL, 2: Stop, 3: SlowR, 4: FastR)
        # We multiply max_velocity by these weights
        weights = [-1.0, -0.3, 0.0, 0.3, 1.0]
        
        if 0 <= action < len(weights):
            target_vel = int(self.max_velocity * weights[action])
        else:
            target_vel = 0
            
        # Send action to Arduino
        command = f"A{target_vel}\n".encode('utf-8')
        self.ser.write(command)
        
        # Read new state back
        obs = self._get_obs()
        cart_pos = obs[0]
        
        # Physical constraints termination check
        terminated = False
        if abs(cart_pos) >= self.cart_limit: 
            terminated = True
            self.ser.write(b"A0\n") # Emergency Stop Motor!
            print("Terminated: Cart reached rail boundaries!")
            
        reward = 0.0 
            
        return obs, float(reward), terminated, False, {}
        
    def close(self):
        # Stop motor immediately
        self.ser.write(b"A0\n")
        self.ser.close()
