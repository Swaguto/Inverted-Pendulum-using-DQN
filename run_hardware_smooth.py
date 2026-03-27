import time
import torch
import numpy as np
from gym_inverted_pendulum import FCQ
from hardware_env import HardwarePendulumEnv

def run_hardware_smooth():
    print("--- Starting Smooth Inverted Pendulum Evaluation (5-Action) ---")
    
    port = 'COM3' 
    baudrate = 250000
    INVERT_MOTOR = True 
    
    try:
        env = HardwarePendulumEnv(port=port, baudrate=baudrate)
    except Exception as e:
        print(f"Error: {e}")
        return
        
    # Load Smooth Model
    model_path = "saved_models/dqn_smooth/model_smooth_final.pth"
    print(f"Loading {model_path}...")
    try:
        model = torch.load(model_path, map_location=torch.device('cpu'), weights_only=False)
        model.eval()
        print("Smooth Model loaded successfully.")
    except Exception as e:
        print(f"Failed to load smooth model: {e}")
        env.close()
        return

    try:
        print("\nReady! Hold at 0° (bottom), then lift to 180°.")
        time.sleep(3)
        state, _ = env.reset()
        step = 0
        
        while True:
            state_tensor = torch.tensor(state, dtype=torch.float32, device=model.device)
            with torch.no_grad():
                q_values = model(state_tensor).detach().cpu().numpy()
            
            action_idx = np.argmax(q_values)
            
            # Application of inversion logic if needed
            if INVERT_MOTOR:
                # 0:FastL -> 4:FastR, 1:SlowL -> 3:SlowR, 2:Stop -> 2:Stop
                actual_action = 4 - action_idx 
            else:
                actual_action = action_idx
                
            next_state, reward, terminal, truncated, _ = env.step(actual_action)
            state = next_state
            step += 1
            
            if step % 50 == 0:
                display_deg = (state[2] + np.pi) * 180 / np.pi
                print(f"Step {step} | Angle: {display_deg:.1f}° | Action: {action_idx}")
            
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        env.close()

if __name__ == "__main__":
    run_hardware_smooth()
