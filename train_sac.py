import os
import numpy as np
import torch
from pendulum_env import InvertedPendulumEnv
from sac_agent import SACAgent

def main():
    env = InvertedPendulumEnv()
    input_dims = env.observation_space.shape[0]
    n_actions = env.action_space.shape[0]
    
    agent = SACAgent(input_dims=input_dims, n_actions=n_actions, env=env, 
                     batch_size=256, reward_scale=2)
    
    n_episodes = 500
    save_dir = "saved_models/sac"
    os.makedirs(save_dir, exist_ok=True)
    
    best_score = -np.inf
    episode_scores = []
    
    print("Starting SAC Training...")
    for i in range(n_episodes):
        observation, _ = env.reset()
        done = False
        score = 0
        while not done:
            action = agent.choose_action(observation)
            observation_, reward, terminated, truncated, info = env.step(action)
            done = terminated or truncated
            agent.remember(observation, action, reward, observation_, done)
            agent.learn()
            observation = observation_
            score += reward
        
        episode_scores.append(score)
        avg_score = np.mean(episode_scores[-10:])
        
        if avg_score > best_score:
            best_score = avg_score
            torch.save(agent.actor.state_dict(), os.path.join(save_dir, "actor_best.pth"))
            torch.save(agent.critic_1.state_dict(), os.path.join(save_dir, "critic_1_best.pth"))
            torch.save(agent.critic_2.state_dict(), os.path.join(save_dir, "critic_2_best.pth"))
        
        if i % 10 == 0:
            print(f"Episode {i} | Score: {score:.2f} | Avg Score (10): {avg_score:.2f}")

    print("Training Complete.")
    torch.save(agent.actor.state_dict(), os.path.join(save_dir, "actor_final.pth"))

if __name__ == "__main__":
    main()
