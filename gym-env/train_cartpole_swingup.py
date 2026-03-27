import math
import random
import matplotlib.pyplot as plt
from collections import namedtuple, deque
from itertools import count

import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
import numpy as np

from cartpole_swingup_env import CartPoleSwingUpEnv

# ── Environment ───────────────────────────────────────────────────────────────
env = CartPoleSwingUpEnv()

# ── Device ────────────────────────────────────────────────────────────────────
device = torch.device(
    "cuda" if torch.cuda.is_available() else
    "mps" if torch.backends.mps.is_available() else
    "cpu"
)
print(f"Using device: {device}")

# ── Replay buffer ─────────────────────────────────────────────────────────────
Transition = namedtuple('Transition', ('state', 'action', 'next_state', 'reward'))

class ReplayMemory:
    def __init__(self, capacity):
        self.memory = deque([], maxlen=capacity)
    def push(self, *args):
        self.memory.append(Transition(*args))
    def sample(self, batch_size):
        return random.sample(self.memory, batch_size)
    def __len__(self):
        return len(self.memory)

# ── DQN Model ─────────────────────────────────────────────────────────────────
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

# ── Hyperparameters ───────────────────────────────────────────────────────────
BATCH_SIZE = 256
GAMMA = 0.99
EPS_START = 1.0
EPS_END = 0.05
EPS_DECAY = 15000 # slower decay for harder task
TAU = 0.005
LR = 1e-4
NUM_EPISODES = 3000
MODEL_PATH = "gym-env/cartpole_swingup_policy_net.pth"

# ── Setup ─────────────────────────────────────────────────────────────────────
n_obs = env.observation_space.shape[0] # 5: [x, x_dot, cos_theta, sin_theta, theta_dot]
n_act = env.action_space.n # 2: [left, right]

policy_net = DQN(n_obs, n_act).to(device)
target_net = DQN(n_obs, n_act).to(device)
target_net.load_state_dict(policy_net.state_dict())

optimizer = optim.AdamW(policy_net.parameters(), lr=LR, amsgrad=True)
memory = ReplayMemory(100_000)
steps_done = 0

def select_action(state):
    global steps_done
    eps = EPS_END + (EPS_START - EPS_END) * math.exp(-steps_done / EPS_DECAY)
    steps_done += 1
    if random.random() > eps:
        with torch.no_grad():
            return policy_net(state).max(1).indices.view(1, 1)
    return torch.tensor([[env.action_space.sample()]], device=device, dtype=torch.long)

def optimize_model():
    if len(memory) < BATCH_SIZE:
        return
    transitions = memory.sample(BATCH_SIZE)
    batch = Transition(*zip(*transitions))

    non_final_mask = torch.tensor([s is not None for s in batch.next_state], device=device, dtype=torch.bool)
    non_final_next = torch.cat([s for s in batch.next_state if s is not None])

    s = torch.cat(batch.state)
    a = torch.cat(batch.action)
    r = torch.cat(batch.reward)

    q_vals = policy_net(s).gather(1, a)
    next_v = torch.zeros(BATCH_SIZE, device=device)
    with torch.no_grad():
        next_v[non_final_mask] = target_net(non_final_next).max(1).values
    expected = (next_v * GAMMA) + r

    loss = nn.SmoothL1Loss()(q_vals, expected.unsqueeze(1))
    optimizer.zero_grad()
    loss.backward()
    torch.nn.utils.clip_grad_value_(policy_net.parameters(), 100)
    optimizer.step()

# ── Training Loop ─────────────────────────────────────────────────────────────
episode_rewards = []
print(f"Training CartPole Swing-Up for {NUM_EPISODES} episodes...")

try:
    for ep in range(NUM_EPISODES):
        obs, _ = env.reset()
        state = torch.tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
        total_reward = 0.0

        for t in count():
            action_idx = select_action(state)
            obs, reward, terminated, truncated, _ = env.step(action_idx.item())
            total_reward += reward
            done = terminated or truncated

            next_state = None if terminated else torch.tensor(obs, dtype=torch.float32, device=device).unsqueeze(0)
            memory.push(state, action_idx, next_state, torch.tensor([reward], device=device, dtype=torch.float32))
            state = next_state
            
            optimize_model()

            # soft-update target
            tsd, psd = target_net.state_dict(), policy_net.state_dict()
            for k in psd:
                tsd[k] = psd[k] * TAU + tsd[k] * (1 - TAU)
            target_net.load_state_dict(tsd)

            if done:
                break

        episode_rewards.append(total_reward)
        if (ep + 1) % 10 == 0:
            avg = sum(episode_rewards[-10:]) / 10
            print(f"Episode {ep+1:4d} | Avg Reward (last 10): {avg:8.2f} | Steps: {steps_done}")

        # Periodic save every 100 episodes
        if (ep + 1) % 100 == 0:
            torch.save(policy_net.state_dict(), MODEL_PATH)
            print(f"Periodic model save at episode {ep+1} -> {MODEL_PATH}")

except KeyboardInterrupt:
    print("\nTraining interrupted by user. Saving current model...")

# ── Save ──────────────────────────────────────────────────────────────────────
torch.save(policy_net.state_dict(), MODEL_PATH)
print(f"Model saved -> {MODEL_PATH}")
