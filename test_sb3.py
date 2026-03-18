import time
from stable_baselines3 import PPO
from pendulum_env import InvertedPendulumEnv

def test():
    # Create environment in GUI mode for visualization
    env = InvertedPendulumEnv(render_mode="human")
    
    # Load the trained model
    model = PPO.load("saved_models/ppo_pendulum")
    
    print("Testing trained PPO agent...")
    for episode in range(5):
        obs, _ = env.reset()
        done = False
        score = 0
        step = 0
        
        while not done:
            # Predict action from observation
            action, _states = model.predict(obs, deterministic=True)
            
            # Step environment
            obs, reward, terminated, truncated, info = env.step(action)
            score += reward
            step += 1
            done = terminated or truncated
            
            # Slow down for visualization
            time.sleep(1/60.)
            
            if step % 60 == 0:
                print(f"Episode {episode+1}, Step {step}, Score: {score}")
        
        print(f"Episode {episode+1} finished with score: {score}")
    
    env.close()

if __name__ == "__main__":
    test()
