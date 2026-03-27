import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import os
import time
import matplotlib.pyplot as plt
from hardware_env import HardwarePendulumEnv

# ── DQN Model (Hardware-Focused) ──────────────────────────────────────────────
class DQN(nn.Module):
    def __init__(self, n_obs, n_act):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(n_obs, 512), nn.ReLU(),
            nn.Linear(512, 256), nn.ReLU(),
            nn.Linear(256, n_act)
        )
    def forward(self, x):
        return self.net(x)

# ── Real-Time Plotting ────────────────────────────────────────────────────────
plt.ion() # interactive plot mode
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_title("Hardware Pendulum: Reward Evolution")
ax.set_xlabel("Episode")
ax.set_ylabel("Total Reward")
line, = ax.plot([], [], label="Episode Reward", color="blue", alpha=0.3)
avg_line, = ax.plot([], [], label="Avg (Last 10)", color="red", linewidth=2)
ax.legend()
plt.grid(True)

def update_plot(rewards):
    if len(rewards) == 0: return
    x = np.arange(1, len(rewards) + 1)
    line.set_data(x, rewards)
    if len(rewards) >= 10:
        # Running average over last 10
        avgs = [np.mean(rewards[max(0, i-10):i+1]) for i in range(len(rewards))]
        avg_line.set_data(x, avgs)
    else:
        avg_line.set_data(x, rewards)
    
    ax.relim()
    ax.autoscale_view()
    plt.pause(0.01)

# ── Training Loop ─────────────────────────────────────────────────────────────
def train_hardware():
    # 1. Initialize Hardware Environment
    # Update COM port as needed (Check Device Manager)
    env = HardwarePendulumEnv(port='COM3', baudrate=115200)
    
    # 2. Hyperparameters
    lr = 1e-4
    gamma = 0.99
    epsilon = 0.1 # Real hardware: low exploration for safety
    
    n_obs = env.observation_space.shape[0]
    n_act = env.action_space.n
    
    policy_net = DQN(n_obs, n_act)
    optimizer = optim.Adam(policy_net.parameters(), lr=lr)
    
    episode_rewards = []
    
    print("--- HARDWARE TRAINING STARTING ---")
    print("Ensure the cart is centered and the pendulum is at the BOTTOM.")
    
    try:
        for ep in range(1000):
            obs, _ = env.reset()
            total_reward = 0
            
            # Episodes in real life: keep them short to avoid overheating
            for t in range(500):
                # Epsilon-Greedy
                if np.random.rand() < epsilon:
                    action = env.action_space.sample()
                else:
                    state_t = torch.tensor(obs, dtype=torch.float32).unsqueeze(0)
                    with torch.no_grad():
                        action = torch.argmax(policy_net(state_t)).item()
                
                # EXECUTE ON HARDWARE
                obs, reward, terminated, truncated, _ = env.step(action)
                total_reward += reward
                
                if terminated or truncated:
                    break
            
            episode_rewards.append(total_reward)
            update_plot(episode_rewards)
            
            print(f"Episode {ep} | Reward: {total_reward:.2f}")
            
    except KeyboardInterrupt:
        print("\nStopping hardware...")
    
    env.close()
    plt.ioff()
    plt.show()

if __name__ == "__main__":
    train_hardware()
