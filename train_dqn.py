import pybullet as p
import torch
import torch.optim as optim
import numpy as np
import pybullet_data
import time
from itertools import count
import math
import os

from gym_inverted_pendulum import FCQ
from gym_inverted_pendulum import ReplayBuffer

class pybulletDQN():
    def __init__(self, replay_buffer, online_model, target_model, optimizer, update_freq, epochs = 40, gamma = 0.99):
        self.replay_buffer = replay_buffer
        self.online_model = online_model
        self.target_model = target_model
        self.optimizer = optimizer
        self.update_freq = update_freq
        self.gamma = gamma
        self.epoch = epochs

    def choose_action_egreedy(self, state, eps):
        state = torch.tensor(state, dtype=torch.float32, device=self.online_model.device)
        with torch.no_grad():
            q = self.online_model(state).detach().cpu().data.numpy().squeeze()
        if np.random.rand() > eps:
            action = np.argmax(q)
        else:
            action = np.random.randint(self.online_model.output_dim)
        return action
    
    def choose_action_greedy(self, state):
        state = torch.tensor(state, dtype=torch.float32, device=self.online_model.device)
        with torch.no_grad():
            q = self.online_model(state).cpu().detach().numpy()
        action = np.argmax(q)
        return action
    
    def soft_update_weights(self, tau=1.0):
        for target_param, online_param in zip(self.target_model.parameters(), self.online_model.parameters()):
            target_param.data.copy_(tau * online_param.data + (1.0 - tau) * target_param.data)

    def learn(self):
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
    
    def eval(self, robotId, max_steps = 500, randomize_val = 0.1, friction = 0.005):
        reset_joint_swingup(robotId, randomize_val=randomize_val, friction=friction)
        state = get_state(robotId)
        episode_score = 0
        terminal = False
        truncated = False

        for step in count():
            state = get_state(robotId)
            if state[0] > 0.3 or state[0] < -0.3:
                reward = -100
                terminal = True
            else:
                reward = math.cos(state[2]) - 0.002 * (state[3]) ** 2 - 0.5 * (state[0] ** 2)

            if step > max_steps:
                truncated = True
            
            if step > 0:
                episode_score += reward

            if terminal or truncated:
                action = -1
                send_action(robotId, action)
                break

            action = self.choose_action_greedy(state)
            send_action(robotId, action)
            p.stepSimulation()
        return episode_score

def reset_joint_swingup(robotId, randomize_val=0.1, friction=0.005):
    randomize_cart = np.random.uniform(-randomize_val, randomize_val)
    # Start near PI (bottom) for swing-up
    randomize_pole = np.random.uniform(-0.1, 0.1)
    target_pole = 3.14159 + randomize_pole
    
    p.resetJointState(bodyUniqueId=robotId, jointIndex=0, targetValue=randomize_cart)
    p.resetJointState(bodyUniqueId=robotId, jointIndex=1, targetValue=target_pole)
    
    p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=1, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=friction)
    p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=0, force=0)

def get_state(robotId):
    cart_state = p.getJointState(robotId, 0)
    pole_state = p.getJointState(robotId, 1)
    
    # Normalize pole angle to [-pi, pi]
    pole_angle = pole_state[0]
    pole_angle = (pole_angle + np.pi) % (2 * np.pi) - np.pi
    
    return np.array([cart_state[0], cart_state[1], pole_angle, pole_state[1]], dtype=np.float32)

def send_action(robotId, action):
    force_limit = 50.0 # High force limit for cart
    if action == 0:
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=-5.0, force=force_limit)
    elif action == 1:
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=5.0, force=force_limit)
    elif action == -1:
        p.setJointMotorControl2(bodyUniqueId=robotId, jointIndex=0, controlMode=p.VELOCITY_CONTROL, targetVelocity=0.0, force=force_limit)

def main():
    epochs = 40
    episodes = 2500
    max_steps = 500

    learning_rate = 0.0005
    batch_size = 256
    warmup_batches = 5
    update_freq = 10
    min_steps_to_learn = warmup_batches * batch_size

    replay_buffer = ReplayBuffer(batch_size=batch_size, max_size=100000)

    num_actions = 2
    num_states = 4
    
    save_dir = "saved_models/dqn_ref"
    os.makedirs(save_dir, exist_ok=True)
    
    online_model = FCQ(num_states, num_actions, (256, 128))
    target_model = FCQ(num_states, num_actions, (256, 128))
    optimizer = optim.RMSprop(online_model.parameters(), lr=learning_rate)

    agent = pybulletDQN(replay_buffer, online_model, target_model, optimizer, update_freq, epochs=epochs, gamma=0.99)
    agent.soft_update_weights()

    eval_interval = 25
    decay_steps = episodes * 0.8
    eval_scores = []
    episode_scores = []

    # Connection
    physicsClient = p.connect(p.DIRECT) # Fast training
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    friction = 0.01 # Slightly higher friction for stabilization
    randomize_val = 0.05
    p.loadURDF("plane.urdf", [0, 0, 0])

    robotId_path = "robot/robot.urdf"
    robotId = p.loadURDF(robotId_path, [0, 0, 1.0], useFixedBase=True)

    for e in range(episodes):
        reset_joint_swingup(robotId, randomize_val=randomize_val, friction=friction)
        state = get_state(robotId)
        episode_score = 0
        eps = max(1-e/decay_steps, 0.05)
        terminal = False
        truncated = False

        for step in count():
            action = agent.choose_action_egreedy(state, eps)
            send_action(robotId, action)
            
            p.stepSimulation()
            
            prev_state = state.copy()
            state = get_state(robotId)
            
            if state[0] > 0.3 or state[0] < -0.3:
                reward = -100
                terminal = True
            else:
                reward = math.cos(state[2]) - 0.002 * (state[3]) ** 2 - 0.5 * (state[0] ** 2)

            if step > max_steps:
                truncated = True
            
            if step > 0:
                experience = (prev_state, action, reward, state, terminal)
                agent.replay_buffer.store(experience)
                episode_score += reward
            
            if terminal or truncated:
                action = -1
                send_action(robotId, action)
                episode_scores.append(episode_score)
                
                if len(agent.replay_buffer) > min_steps_to_learn:
                    for _ in range(step):
                        agent.learn()
                
                if e % update_freq == 0 and e > 1:
                    agent.soft_update_weights()
                    
                break

        if e % 10 == 0 and e > 1:
            print(f"Episode: {e}/{episodes}, Rolling mean: {int(np.mean(episode_scores[-50:]))}, Epsilon: {eps:.2f}")

        if e % eval_interval == 0 and e > 1:
            eval_score = agent.eval(robotId=robotId, max_steps=max_steps, friction=friction)
            eval_scores.append(eval_score)
            eval_rolling_mean = int(np.mean(eval_scores[-5:]))
            print(f"  -> Eval score: {eval_score:.1f}, Rolling mean: {eval_rolling_mean}")
            
            if eval_rolling_mean > 200:
                print("Solved!")
                torch.save(agent.online_model, os.path.join(save_dir, "model_final.pth"))
                break

    p.disconnect()
    torch.save(agent.online_model, os.path.join(save_dir, "model_final.pth"))

if __name__ == "__main__":
    main()
