import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
import random
from collections import deque

class FCQ(nn.Module):
    """
    Fully Connected Q-Network.
    """
    def __init__(self, input_dim, output_dim, hidden_dims=(256, 128)):
        super(FCQ, self).__init__()
        self.input_dim = input_dim
        self.output_dim = output_dim
        
        # Build network layers
        layers = []
        last_dim = input_dim
        for h_dim in hidden_dims:
            layers.append(nn.Linear(last_dim, h_dim))
            layers.append(nn.ReLU())
            last_dim = h_dim
        layers.append(nn.Linear(last_dim, output_dim))
        
        self.model = nn.Sequential(*layers)
        
        # Setup device
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.to(self.device)

    def forward(self, state):
        if not isinstance(state, torch.Tensor):
            state = torch.tensor(state, dtype=torch.float32, device=self.device)
        return self.model(state)

class ReplayBuffer:
    """
    Simple Experience Replay Buffer.
    """
    def __init__(self, max_size=100000, batch_size=64):
        self.buffer = deque(maxlen=max_size)
        self.batch_size = batch_size

    def store(self, experience):
        self.buffer.append(experience)

    def draw_samples(self):
        samples = random.sample(self.buffer, self.batch_size)
        
        states, actions, rewards, next_states, terminals = zip(*samples)
        
        return (
            np.array(states),
            np.array(actions).reshape(-1, 1),
            np.array(rewards).reshape(-1, 1),
            np.array(next_states),
            np.array(terminals).reshape(-1, 1)
        )

    def __len__(self):
        return len(self.buffer)
