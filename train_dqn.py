import torch
import torch.optim as optim
import numpy as np
import time
import os
import math
from itertools import count
import matplotlib.pyplot as plt

from pendulum_env import InvertedPendulumEnv
from gym_inverted_pendulum import FCQ, ReplayBuffer

class pybulletDQN():
    def __init__(self, replay_buffer, online_model, target_model, optimizer, update_freq, gamma=0.99):
        self.replay_buffer = replay_buffer
        self.online_model = online_model
        self.target_model = target_model
        self.optimizer = optimizer
        self.update_freq = update_freq
        self.gamma = gamma

    def choose_action_egreedy(self, state, eps):
        state = torch.tensor(state, dtype=torch.float32, device=self.online_model.device).unsqueeze(0)
        with torch.no_grad():
            q = self.online_model(state).detach().cpu().numpy().squeeze()

        if np.random.rand() > eps:
            action = np.argmax(q)
        else:
            action = np.random.randint(self.online_model.output_dim)
        return action
    
    def soft_update_weights(self, tau=1.0):
        for target_param, online_param in zip(self.target_model.parameters(), self.online_model.parameters()):
            target_param.data.copy_(tau * online_param.data + (1.0 - tau) * target_param.data)

    def learn(self):
        if len(self.replay_buffer) < self.replay_buffer.batch_size:
            return 0
        states, actions, rewards, next_states, terminals = self.replay_buffer.draw_samples()
        states = torch.tensor(states, dtype=torch.float32, device=self.online_model.device)
        actions = torch.tensor(actions, dtype=torch.int64, device=self.online_model.device)
        rewards = torch.tensor(rewards, dtype=torch.float32, device=self.online_model.device)
        next_states = torch.tensor(next_states, dtype=torch.float32, device=self.online_model.device)
        terminals = torch.tensor(terminals, dtype=torch.float32, device=self.online_model.device)
        qsa_next_max = self.target_model(next_states).detach().max(1)[0].unsqueeze(1)
        yj = rewards + (self.gamma * qsa_next_max * (1 - terminals))
        qsa = self.online_model(states).gather(1, actions)
        loss = torch.nn.functional.mse_loss(qsa, yj)
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        return loss.item()

def train():
    episodes = 2000
    max_steps = 500
    learning_rate = 0.001
    batch_size = 128
    update_freq = 10
    save_dir = "saved_models"
    os.makedirs(save_dir, exist_ok=True)

    env = InvertedPendulumEnv(render_mode=None)
    num_actions = 3
    num_states = 4
    
    def env_step_discrete(env, action_idx):
        forces = [-7.0, 7.0, 0.0] # Realistic force for balancing
        return env.step([forces[action_idx]])

    online_model = FCQ(num_states, num_actions)
    target_model = FCQ(num_states, num_actions)
    optimizer = optim.Adam(online_model.parameters(), lr=learning_rate)
    replay_buffer = ReplayBuffer(batch_size=batch_size)
    agent = pybulletDQN(replay_buffer, online_model, target_model, optimizer, update_freq)
    agent.soft_update_weights()

    episode_scores = []
    # VERY AGGRESSIVE DECAY to force learning quickly
    decay_steps = 300 

    for e in range(episodes):
        state, _ = env.reset()
        episode_score = 0
        eps = max(1 - e / decay_steps, 0.02)

        for step in count():
            action_idx = agent.choose_action_egreedy(state, eps)
            next_state, reward, terminated, truncated, _ = env_step_discrete(env, action_idx)
            agent.replay_buffer.store((state, action_idx, reward, next_state, terminated))
            state = next_state
            episode_score += reward
            if terminated or truncated or step >= max_steps:
                break

        episode_scores.append(episode_score)
        if (e+1) % 10 == 0:
            avg_score = np.mean(episode_scores[-50:])
            print(f"Episode: {e+1}/{episodes}, Score: {episode_score:.1f}, Avg: {avg_score:.1f}, Eps: {eps:.2f}")
        
        if len(agent.replay_buffer) > 1000:
            for _ in range(16): 
                agent.learn()
        
        if e % update_freq == 0:
            agent.soft_update_weights()
            
        if len(episode_scores) > 100 and np.mean(episode_scores[-50:]) > 450:
            print("Environment Solved!")
            torch.save(online_model, f"{save_dir}/model_final.pth")
            break

    env.close()
    torch.save(online_model, f"{save_dir}/model_final.pth")

if __name__ == "__main__":
    train()
