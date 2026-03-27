import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Normal

class CriticNetwork(nn.Module):
    def __init__(self, input_dim, n_actions, name='critic', hidden_dims=(256, 256)):
        super(CriticNetwork, self).__init__()
        self.input_dim = input_dim
        self.n_actions = n_actions
        self.name = name

        self.fc1 = nn.Linear(input_dim + n_actions, hidden_dims[0])
        self.fc2 = nn.Linear(hidden_dims[0], hidden_dims[1])
        self.q = nn.Linear(hidden_dims[1], 1)

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state, action):
        action_value = self.fc1(torch.cat([state, action], dim=1))
        action_value = F.relu(action_value)
        action_value = self.fc2(action_value)
        action_value = F.relu(action_value)
        q = self.q(action_value)
        return q

class ActorNetwork(nn.Module):
    def __init__(self, input_dim, n_actions, name='actor', hidden_dims=(256, 256), max_action=1.0):
        super(ActorNetwork, self).__init__()
        self.input_dim = input_dim
        self.n_actions = n_actions
        self.name = name
        self.max_action = max_action
        self.reparam_noise = 1e-6

        self.fc1 = nn.Linear(input_dim, hidden_dims[0])
        self.fc2 = nn.Linear(hidden_dims[0], hidden_dims[1])
        self.mu = nn.Linear(hidden_dims[1], n_actions)
        self.sigma = nn.Linear(hidden_dims[1], n_actions)

        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        self.to(self.device)

    def forward(self, state):
        prob = self.fc1(state)
        prob = F.relu(prob)
        prob = self.fc2(prob)
        prob = F.relu(prob)

        mu = self.mu(prob)
        sigma = self.sigma(prob)
        sigma = torch.clamp(sigma, min=self.reparam_noise, max=1)

        return mu, sigma

    def sample_normal(self, state, reparameterize=True):
        mu, sigma = self.forward(state)
        probabilities = Normal(mu, sigma)

        if reparameterize:
            actions = probabilities.rsample()
        else:
            actions = probabilities.sample()

        action = torch.tanh(actions) * torch.tensor(self.max_action).to(self.device)
        log_probs = probabilities.log_prob(actions)
        log_probs -= torch.log(1 - action.pow(2) + self.reparam_noise)
        log_probs = log_probs.sum(1, keepdim=True)

        return action, log_probs
