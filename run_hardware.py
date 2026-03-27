import time
import torch
import numpy as np

# We must import FCQ to successfully unpickle the saved model structure
from gym_inverted_pendulum import FCQ
from hardware_env import HardwarePendulumEnv

def run_hardware_evaluation():
    print("--- Starting Real-World Inverted Pendulum evaluation ---")
    
    # Initialize the Serial Environment
    # NOTE: You may need to change 'COM3' to whatever port your Arduino is on
    port = 'COM3' 
    baudrate = 250000
    
    # SET THIS TO True IF THE MOTOR PUSHES THE WRONG WAY
    INVERT_MOTOR = True 
    
    try:
        env = HardwarePendulumEnv(port=port, baudrate=baudrate)
    except Exception as e:
        print(f"Failed to connect to Arduino on {port}. Please check your connection and port name.")
        print("Error details:", e)
        return
        
    print("Environment initialized via Serial.")
    
    # Load Model
    model_path = "saved_models/dqn_ref/model_final.pth"
    print(f"Loading weights from {model_path}...")
    try:
        model = torch.load(model_path, map_location=torch.device('cpu'), weights_only=False)
        model.eval()
        print("Model loaded successfully.")
    except Exception as e:
        print(f"Failed to load model. Ensure the path is correct and training has finished.")
        print("Error details:", e)
        env.close()
        return

    # Evaluation Loop
    try:
        print("\nReady! Hold the pendulum upright or click reset on your Arduino, then let go.")
        print("Starting in 3 seconds...")
        time.sleep(3)
        
        state, _ = env.reset()
        terminal = False
        step = 0
        
        
        while not terminal:
            # 1. State to Tensor
            state_tensor = torch.tensor(state, dtype=torch.float32, device=model.device)
            
            # 2. Forward Pass
            with torch.no_grad():
                q_values = model(state_tensor).detach().cpu().numpy()
                
            # 3. Action Selection (Greedy)
            action_idx = np.argmax(q_values)
            
            # 4. Step Environment 
            actual_action = action_idx
            if INVERT_MOTOR:
                actual_action = 1 - action_idx # flip 0 to 1 and vice versa
                
            next_state, reward, terminal, truncated, _ = env.step(actual_action)
            
            state = next_state
            step += 1
            
            # Print brief status every 50 loops
            if step % 50 == 0:
                # Convert back to degrees for user display (0 at bottom, 180 at top)
                # state[2] is internal angle (0 = top). 
                # Display degree = (state[2] + pi) * 180 / pi
                display_deg = (state[2] + np.pi) * 180 / np.pi
                print(f"Step {step} | Pos: {state[0]:.2f}m | Vel: {state[1]:.2f} | Angle: {display_deg:.1f}°")
                if abs(state[0]) < 0.001 and display_deg < 0.1:
                    print("  [Note: Encoder readings are currently zero. Try moving the hardware by hand.]")
            
            # Brief sleep to match model inference rate trained at (1/60th or 1/240th usually). 
            # Hardware serial inherently provides some delay, but we ensure we don't overkill.
            # time.sleep(1/60.0) 
            
        print("Terminal state reached (cart hit boundaries). Stopping.")
        
    except KeyboardInterrupt:
        print("\nUser manually interrupted!")
        
    finally:
        print("Shutting down safely.")
        env.close()

if __name__ == "__main__":
    run_hardware_evaluation()
