import gymnasium as gym
import pybullet as p
import pybullet_data
import numpy as np
import time

class InvertedPendulumEnv(gym.Env):
    """
    Inverted Pendulum for PyBullet with shaped angle-based reward.
    The agent must keep the pole near vertical; the reward is proportional
    to cos(angle), so staying perfectly upright gives the maximum reward.
    """
    def __init__(self, render_mode=None):
        super(InvertedPendulumEnv, self).__init__()
        
        self.render_mode = render_mode
        if render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Continuous force action on the cart (N)
        # Reduced max force for more realistic hardware limits
        self.max_force = 20.0
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(1,), dtype=np.float32)
        
        # Observations: [cart_pos, cart_vel, cos(pole_angle), sin(pole_angle), pole_vel]
        # We use cos/sin to avoid angle discontinuity (swinging past 180 deg)
        self.observation_space = gym.spaces.Box(
            low=np.array([-1.0, -1.0, -1.0, -1.0, -1.0]),
            high=np.array([1.0, 1.0, 1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        self.urdf_path = "robot/robot.urdf"
        self.robotId = None
        self.cart_joint_index = 0
        self.pole_joint_index = 1
        
        # Physical limits from URDF
        # Physical limits from URDF (Matching ~25 inch rail: +/- 12.5 inch = 0.31m)
        self.cart_limit = 0.3

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        if self.robotId is None:
            p.resetSimulation()
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")
            # Load with fixed base at 1.0m height
            self.robotId = p.loadURDF(self.urdf_path, [0, 0, 1.0], useFixedBase=True)
            p.changeDynamics(self.robotId, self.pole_joint_index, jointDamping=0.01) # Slightly more damping
            p.setJointMotorControl2(self.robotId, self.pole_joint_index, p.VELOCITY_CONTROL, force=0)
        
        # Fast reset — reset joint states only
        p.resetJointState(self.robotId, self.cart_joint_index, 0.0, 0.0)
        
        # START AT BOTTOM (PI)
        # Add a bit of random tilt to start the swing
        initial_angle = np.pi + self.np_random.uniform(low=-0.2, high=0.2)
        p.resetJointState(self.robotId, self.pole_joint_index, initial_angle, 0.0)
        
        return self._get_obs(), {}

    def _get_obs(self):
        cart_state = p.getJointState(self.robotId, self.cart_joint_index)
        pole_state = p.getJointState(self.robotId, self.pole_joint_index)
        
        theta = pole_state[0]
        
        obs = np.array([
            cart_state[0] / self.cart_limit,    # cart position normalized to [-1,1]
            cart_state[1] / 2.0,               # cart velocity
            np.cos(theta),                     # cos(angle)
            np.sin(theta),                     # sin(angle)
            pole_state[1] / 10.0               # pole angular velocity
        ], dtype=np.float32)
        return obs

    def step(self, action):
        if not p.isConnected():
            return np.zeros(5, dtype=np.float32), 0, True, True, {}
            
        # Action is normalized [-1, 1], scale to max_force
        applied_force = action[0] * self.max_force
        
        p.setJointMotorControl2(
            bodyIndex=self.robotId,
            jointIndex=self.cart_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=float(np.clip(applied_force, -self.max_force, self.max_force))
        )
        
        # 1 simulation step per agent step (240 Hz)
        p.stepSimulation()
        
        obs = self._get_obs()
        cart_pos_norm, cart_vel, cos_theta, sin_theta, pole_vel = obs
        
        # REWARD FUNCTION: Aligned with reference SAC project
        # Reference uses: cos(theta) - 0.001 * (theta_dot**2) - 0.1 * abs(x)
        
        # 1. Base reward is cos(theta) (upright = 1.0, bottom = -1.0)
        reward = float(cos_theta)
        
        # 2. Penalty for angular velocity (stabilization)
        reward -= 0.001 * (pole_vel ** 2)
        
        # 3. Penalty for cart distance from center
        reward -= 0.1 * abs(cart_pos_norm)
        
        # Termination: cart hits track limits
        terminated = bool(np.abs(cart_pos_norm) > 1.0)
        
        if terminated:
            reward = -100.0 # Heavy penalty for hitting the wall
            
        return obs, float(reward), terminated, False, {}

    def close(self):
        p.disconnect()
