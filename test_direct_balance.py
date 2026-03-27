import time
import numpy as np
from hardware_env import HardwarePendulumEnv

def test_direct_p_control():
    print("--- Starting Direct P-Control Hardware Test ---")
    print("This script will attempt to 'chase' the pendulum without the AI.")
    
    # 1. Initialize Environment
    # Ensure COM3 is correct!
    try:
        env = HardwarePendulumEnv(port='COM3', baudrate=115200)
    except Exception as e:
        print(f"Error: {e}")
        return

    # 2. Control Parameters
    # Tune Kp (Proportional Gain). 
    # If the cart moves AWAY from the fall, change Kp to negative!
    Kp = 40000.0 
    
    print("\nStarting in 3 seconds. Hold current centered position...")
    time.sleep(3)
    
    obs, _ = env.reset()
    
    try:
        while True:
            # obs[2] is the wrapped pole angle (normalized to [-pi, pi])
            # Upright is 0.0 rad.
            angle = obs[2]
            
            # Simple P-control: velocity proportional to angle error
            # If falling right (angle > 0), move right (velocity > 0)
            target_vel = int(Kp * angle)
            
            # Constrain velocity for safety
            target_vel = max(min(target_vel, 15000), -15000)
            
            # Only command if the angle is significant (ignore hanging down)
            if abs(angle) > 1.5: 
                # Pendulum is hanging down, stay still
                target_vel = 0
            
            # Send command directly to environment's serial
            env.ser.write(f"A{target_vel}\n".encode())
            
            # Get new observation directly
            obs = env._get_obs()
            
            # Manual boundary check
            if abs(obs[0]) >= env.cart_limit:
                print("Boundary reached! Stopping.")
                env.ser.write(b"A0\n")
                break
                
            time.sleep(0.01) # 100Hz control loop
            
    except KeyboardInterrupt:
        print("\nStopped by user.")
    finally:
        env.close()

if __name__ == "__main__":
    test_direct_p_control()
