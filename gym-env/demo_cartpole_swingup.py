import torch
import torch.nn as nn
import numpy as np
import time
from cartpole_swingup_env import CartPoleSwingUpEnv

# ── DQN Model (Must match training) ───────────────────────────────────────────
class DQN(nn.Module):
    def __init__(self, n_obs, n_act):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_obs, 256), nn.ReLU(),
            nn.Linear(256, 256), nn.ReLU(),
            nn.Linear(256, 128), nn.ReLU(),
            nn.Linear(128, n_act)
        )
    def forward(self, x):
        return self.net(x)

# ── Setup ─────────────────────────────────────────────────────────────────────
MODEL_PATH = "gym-env/cartpole_swingup_policy_net.pth"
env = CartPoleSwingUpEnv(render_mode="human")
n_obs = env.observation_space.shape[0]
n_act = env.action_space.n

device = torch.device("cpu") # Demo usually fine on CPU
policy_net = DQN(n_obs, n_act).to(device)

try:
    policy_net.load_state_dict(torch.load(MODEL_PATH, map_location=device, weights_only=True))
    print(f"Loaded model from {MODEL_PATH}")
except FileNotFoundError:
    print(f"Model file {MODEL_PATH} not found. Running with random actions.")

# ── Demo Loop ─────────────────────────────────────────────────────────────────
def run_demo():
    obs, _ = env.reset()
    done = False
    total_reward = 0
    
    while not done:
        state = torch.tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
        
        with torch.no_grad():
            action = policy_net(state).max(1).indices.item()
        
        obs, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward
        done = terminated or truncated
        
        # Note: Since we don't have a 2nd renderer yet, 
        # we'll just print the state to show it's working
        x, x_dot, cos_theta, sin_theta, theta_dot = obs
        theta = np.arctan2(sin_theta, cos_theta)
        print(f"Pos: {x:5.2f} | Angle: {np.degrees(theta):7.1f}° | Reward: {reward:5.2f}", end="\r")
        
        time.sleep(0.02) # Simulating realtime
        
    print(f"\nEpisode finished. Total Reward: {total_reward:.2f}")

if __name__ == "__main__":
    while True:
        run_demo()
        print("\nResetting in 2 seconds...")
        time.sleep(2)
