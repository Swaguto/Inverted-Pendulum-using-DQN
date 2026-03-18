import torch
import numpy as np
import time
from pendulum_env import InvertedPendulumEnv
from gym_inverted_pendulum import FCQ

def test_agent(model_path="saved_models/model_final.pth", num_episodes=5):
    """
    Load the trained model and run it in the environment with rendering.
    """
    print(f"--- Testing Trained Agent: {model_path} ---")
    
    # 1. Setup Environment
    env = InvertedPendulumEnv(render_mode="human")
    
    # 2. Load Model
    if not torch.cuda.is_available():
        # Load GPU model on CPU if necessary
        model = torch.load(model_path, map_location=torch.device('cpu'), weights_only=False)
    else:
        model = torch.load(model_path, weights_only=False)
    
    model.eval()
    
    forces = [-7.0, 7.0, 0.0]  # Actions correspond to [-7N, 7N, 0N]

    for e in range(num_episodes):
        state, _ = env.reset()
        episode_score = 0
        done = False
        
        print(f"Episode {e+1} starting...")
        step = 0
        while not done:
            # Select best action (greedy)
            state_tensor = torch.tensor(state, dtype=torch.float32, device=model.device).unsqueeze(0)
            with torch.no_grad():
                q_values = model(state_tensor).cpu().numpy()
            
            action_idx = np.argmax(q_values)
            
            # Step environment
            state, reward, terminated, truncated, _ = env.step([forces[action_idx]])
            episode_score += reward
            
            # Diagnostic print every 30 steps
            if step % 30 == 0:
                # Denormalize cart pos for logging
                cart_pos = state[0] * 0.26 + 0.04
                print(f"  Step {step}: Action={forces[action_idx]}N, Angle={state[2]:.2f}, Pos={cart_pos:.2f}")
            
            step += 1
            done = terminated or truncated
            
            # Slow down for visual inspection
            time.sleep(1./60.)
            
        print(f"Episode {e+1} finished. Score: {episode_score:.2f}")

    env.close()

if __name__ == "__main__":
    test_agent()
