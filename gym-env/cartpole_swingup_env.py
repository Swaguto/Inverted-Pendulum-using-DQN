import gymnasium as gym
from gymnasium import spaces
import numpy as np
import math

class CartPoleSwingUpEnv(gym.Env):
    """
    Cart-Pole Swing-Up Environment.
    
    Modified from Gymnasium's CartPole-v1:
    1. Starts with the pole hanging down (theta = pi).
    2. No termination based on pole angle (allows full 360-degree rotation).
    3. Reward function based on cos(theta) to encourage upright balance.
    4. State: [x, x_dot, cos_theta, sin_theta, theta_dot]
    """
    
    def __init__(self, render_mode=None):
        super(CartPoleSwingUpEnv, self).__init__()
        
        self.gravity = 9.8
        self.masscart = 1.0
        self.masspole = 0.1
        self.total_mass = (self.masspole + self.masscart)
        self.length = 0.5 # actually half the pole's length
        self.polemass_length = (self.masspole * self.length)
        self.force_mag = 10.0
        self.tau = 0.02 # seconds between state updates
        self.kinematics_integrator = 'euler'

        # Angle at which to fail the episode
        # In this version, we don't fail based on angle, but we need a limit for x
        self.x_threshold = 2.4

        # Observation space: [x, x_dot, cos_theta, sin_theta, theta_dot]
        high = np.array([
            self.x_threshold * 2,
            np.finfo(np.float32).max,
            1.0, 1.0,
            np.finfo(np.float32).max
        ], dtype=np.float32)

        self.action_space = spaces.Discrete(2)
        self.observation_space = spaces.Box(-high, high, dtype=np.float32)

        self.render_mode = render_mode
        self.state = None
        self.steps_beyond_terminated = None
        self.max_episode_steps = 500
        self._step_count = 0

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        # Start at bottom: theta = PI
        # Add a bit of noise to break symmetry
        x = self.np_random.uniform(low=-0.05, high=0.05)
        x_dot = self.np_random.uniform(low=-0.05, high=0.05)
        theta = np.pi + self.np_random.uniform(low=-0.05, high=0.05)
        theta_dot = self.np_random.uniform(low=-0.05, high=0.05)
        
        self.state = (x, x_dot, theta, theta_dot)
        self._step_count = 0
        return self._get_obs(), {}

    def _get_obs(self):
        x, x_dot, theta, theta_dot = self.state
        return np.array([
            x, x_dot, np.cos(theta), np.sin(theta), theta_dot
        ], dtype=np.float32)

    def step(self, action):
        err_msg = f"{action!r} ({type(action)}) invalid"
        assert self.action_space.contains(action), err_msg
        assert self.state is not None, "Call reset before using step()."
        
        x, x_dot, theta, theta_dot = self.state
        force = self.force_mag if action == 1 else -self.force_mag
        costheta = math.cos(theta)
        sintheta = math.sin(theta)

        # Non-linear dynamics
        temp = (force + self.polemass_length * theta_dot**2 * sintheta) / self.total_mass
        thetaacc = (self.gravity * sintheta - costheta * temp) / (self.length * (4.0 / 3.0 - self.masspole * costheta**2 / self.total_mass))
        xacc = temp - self.polemass_length * thetaacc * costheta / self.total_mass

        if self.kinematics_integrator == 'euler':
            x = x + self.tau * x_dot
            x_dot = x_dot + self.tau * xacc
            theta = theta + self.tau * theta_dot
            theta_dot = theta_dot + self.tau * thetaacc
        else: # semi-implicit euler
            x_dot = x_dot + self.tau * xacc
            x = x + self.tau * x_dot
            theta_dot = theta_dot + self.tau * thetaacc
            theta = theta + self.tau * theta_dot

        self.state = (x, x_dot, theta, theta_dot)

        # Reward: encourage keeping the pole upright (cos(theta) -> 1)
        # We use a reward that peaks strongly at the top and penalizes velocity
        # to encourage stable balancing.
        
        # 1. Angle reward: 1.0 at upright, -1.0 at bottom.
        reward = float(costheta)
        
        # 2. Stability penalty: penalize angular and cart velocity
        # This is CRITICAL for staying upright once swung up.
        reward -= 0.05 * (theta_dot**2)
        reward -= 0.05 * (x_dot**2)
        
        # 3. Penalty for cart distance (keep it centered)
        reward -= 0.1 * (x**2)
        
        # 4. Optional: "Survive" bonus if near upright
        if costheta > 0.95:
            reward += 2.0
        
        # Termination: if x goes out of bounds
        terminated = bool(
            x < -self.x_threshold or
            x > self.x_threshold
        )
        
        if terminated:
            reward = -10.0 # penalty for losing control of the cart

        self._step_count += 1
        truncated = bool(self._step_count >= self.max_episode_steps)
        
        return self._get_obs(), reward, terminated, truncated, {}

    def render(self):
        # Optional: Implement visualization using Gymnasium's CartPole rendering or PyBullet
        pass

    def close(self):
        pass
