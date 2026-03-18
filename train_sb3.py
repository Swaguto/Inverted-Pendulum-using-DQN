import os
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from pendulum_env import InvertedPendulumEnv

def train():
    # Single env — our fast reset is already efficient
    env = InvertedPendulumEnv(render_mode=None)
    
    # PPO tuned for continuous cart-pole balancing
    model = PPO(
        "MlpPolicy", env, verbose=1,
        learning_rate=0.0005,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        gae_lambda=0.95,
        clip_range=0.2,
        ent_coef=0.005,   # small entropy bonus to keep exploring
        policy_kwargs=dict(net_arch=[dict(pi=[128, 128], vf=[128, 128])])
    )
    
    print("Starting SB3 PPO Training (1M steps with cos(angle) reward)...")
    model.learn(total_timesteps=1_000_000)
    
    os.makedirs("saved_models", exist_ok=True)
    model.save("saved_models/ppo_pendulum")
    print("Training complete. Model saved to saved_models/ppo_pendulum.zip")
    env.close()

if __name__ == "__main__":
    train()
