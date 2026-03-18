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
        
        if render_mode == "human":
            self.physicsClient = p.connect(p.GUI)
        else:
            self.physicsClient = p.connect(p.DIRECT)
            
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        
        # Continuous force action on the cart
        self.action_space = gym.spaces.Box(low=-30.0, high=30.0, shape=(1,), dtype=np.float32)
        
        # Observations: Normalized [cart_pos, cart_vel, pole_angle, pole_vel]
        self.observation_space = gym.spaces.Box(
            low=np.array([-2.0, -2.0, -2.0, -2.0]),
            high=np.array([2.0, 2.0, 2.0, 2.0]),
            dtype=np.float32
        )
        
        self.urdf_path = "robot/robot.urdf"
        self.robotId = None
        self.cart_joint_index = 0
        self.pole_joint_index = 1

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        
        if self.robotId is None:
            p.resetSimulation()
            p.setGravity(0, 0, -9.81)
            p.loadURDF("plane.urdf")
            self.robotId = p.loadURDF(self.urdf_path, [0, 0, 1.0], useFixedBase=True)
            p.changeDynamics(self.robotId, self.pole_joint_index, jointDamping=0.0)
            p.setJointMotorControl2(self.robotId, self.pole_joint_index, p.VELOCITY_CONTROL, force=0)
        
        # Fast reset — reset joint states only
        p.resetJointState(self.robotId, self.cart_joint_index, 0.0, 0.0)
        # Random initial tilt from 5° to 12° to force active balancing
        sign = self.np_random.choice([-1, 1])
        mag = self.np_random.uniform(low=0.08, high=0.21)  # 4.6° to 12°
        initial_tilt = sign * mag
        p.resetJointState(self.robotId, self.pole_joint_index, initial_tilt, 0.0)
        
        return self._get_obs(), {}

    def _get_obs(self):
        cart_state = p.getJointState(self.robotId, self.cart_joint_index)
        pole_state = p.getJointState(self.robotId, self.pole_joint_index)
        obs = np.array([
            cart_state[0] / 2.0,        # cart position normalized to [-1,1]
            cart_state[1] / 5.0,        # cart velocity
            pole_state[0],              # pole angle (rad) — already small
            pole_state[1] / 5.0         # pole angular velocity
        ], dtype=np.float32)
        return obs

    def step(self, action):
        if not p.isConnected():
            return np.zeros(4, dtype=np.float32), 0, True, True, {}
            
        p.setJointMotorControl2(
            bodyIndex=self.robotId,
            jointIndex=self.cart_joint_index,
            controlMode=p.TORQUE_CONTROL,
            force=float(np.clip(action[0], -30.0, 30.0))
        )
        
        # 1 simulation step per agent step (240 Hz)
        p.stepSimulation()
        
        obs = self._get_obs()
        cart_pos_norm, _, pole_angle, _ = obs
        
        # SHAPED REWARD: cos(angle) gives 1.0 when vertical, drops to 0 at 90°
        # This forces the agent to keep the pole as upright as possible
        reward = np.cos(pole_angle)
        
        # Termination: cart out of range or pole past 45°
        terminated = bool(
            np.abs(cart_pos_norm) > 0.95 or
            np.abs(pole_angle) > 0.785  # 45°
        )
        if terminated:
            reward = -1.0  # Penalty for falling
            
        return obs, reward, terminated, False, {}

    def close(self):
        p.disconnect()
