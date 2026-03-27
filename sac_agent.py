import torch
import torch.nn.functional as F
import numpy as np
from sac_models import ActorNetwork, CriticNetwork
from gym_inverted_pendulum import ReplayBuffer

class SACAgent():
    def __init__(self, input_dims, n_actions, env, alpha=0.0003, beta=0.0003, 
                 gamma=0.99, tau=0.005, batch_size=256, reward_scale=2, max_size=1000000):
        self.gamma = gamma
        self.tau = tau
        self.batch_size = batch_size
        self.n_actions = n_actions
        self.memory = ReplayBuffer(max_size, batch_size)

        self.actor = ActorNetwork(input_dims, n_actions, max_action=env.action_space.high)
        self.critic_1 = CriticNetwork(input_dims, n_actions, name='critic_1')
        self.critic_2 = CriticNetwork(input_dims, n_actions, name='critic_2')
        self.target_critic_1 = CriticNetwork(input_dims, n_actions, name='target_critic_1')
        self.target_critic_2 = CriticNetwork(input_dims, n_actions, name='target_critic_2')

        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=alpha)
        self.critic_1_optimizer = torch.optim.Adam(self.critic_1.parameters(), lr=beta)
        self.critic_2_optimizer = torch.optim.Adam(self.critic_2.parameters(), lr=beta)

        self.scale = reward_scale
        self.update_network_parameters(tau=1)

    def choose_action(self, observation):
        state = torch.tensor([observation], dtype=torch.float32).to(self.actor.device)
        actions, _ = self.actor.sample_normal(state, reparameterize=False)
        return actions.cpu().detach().numpy()[0]

    def remember(self, state, action, reward, new_state, done):
        self.memory.store((state, action, reward, new_state, done))

    def update_network_parameters(self, tau=None):
        if tau is None:
            tau = self.tau

        for target_param, param in zip(self.target_critic_1.parameters(), self.critic_1.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

        for target_param, param in zip(self.target_critic_2.parameters(), self.critic_2.parameters()):
            target_param.data.copy_(target_param.data * (1.0 - tau) + param.data * tau)

    def learn(self):
        if len(self.memory) < self.batch_size:
            return

        state, action, reward, new_state, done = self.memory.draw_samples()

        reward = torch.tensor(reward, dtype=torch.float32).to(self.actor.device)
        done = torch.tensor(done, dtype=torch.bool).to(self.actor.device)
        state_ = torch.tensor(new_state, dtype=torch.float32).to(self.actor.device)
        state = torch.tensor(state, dtype=torch.float32).to(self.actor.device)
        action = torch.tensor(action, dtype=torch.float32).to(self.actor.device)

        with torch.no_grad():
            next_actions, next_log_probs = self.actor.sample_normal(state_, reparameterize=False)
            q1_target = self.target_critic_1(state_, next_actions)
            q2_target = self.target_critic_2(state_, next_actions)
            min_q_target = torch.min(q1_target, q2_target) - self.scale * next_log_probs
            y = reward + self.gamma * (~done) * min_q_target

        # Update Critics
        self.critic_1_optimizer.zero_grad()
        q1 = self.critic_1(state, action)
        critic_1_loss = F.mse_loss(q1, y)
        critic_1_loss.backward()
        self.critic_1_optimizer.step()

        self.critic_2_optimizer.zero_grad()
        q2 = self.critic_2(state, action)
        critic_2_loss = F.mse_loss(q2, y)
        critic_2_loss.backward()
        self.critic_2_optimizer.step()

        # Update Actor
        self.actor_optimizer.zero_grad()
        actions, log_probs = self.actor.sample_normal(state, reparameterize=True)
        q1_new = self.critic_1(state, actions)
        q2_new = self.critic_2(state, actions)
        min_q_new = torch.min(q1_new, q2_new)
        actor_loss = (self.scale * log_probs - min_q_new).mean()
        actor_loss.backward()
        self.actor_optimizer.step()

        self.update_network_parameters()
