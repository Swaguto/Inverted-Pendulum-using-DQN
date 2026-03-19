import pybullet as p
import torch
import numpy as np
import time
from train_dqn import pybulletDQN, get_state, reset_joint_swingup, send_action
from gym_inverted_pendulum import FCQ

def test_model():
    print("--- Testing Trained DQN Agent ---")
    
    # 1. Setup PyBullet GUI
    physicsClient = p.connect(p.GUI)
    import pybullet_data
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    friction = 0.01
    p.loadURDF("plane.urdf", [0, 0, 0])
    robotId = p.loadURDF("robot/robot.urdf", [0, 0, 1.0], useFixedBase=True)

    # 2. Load Model
    model_path = "saved_models/dqn_ref/model_final.pth"
    print(f"Loading {model_path}...")
    model = torch.load(model_path, map_location=torch.device('cpu'), weights_only=False)
    model.eval()

    for e in range(5):
        reset_joint_swingup(robotId, randomize_val=0.05, friction=friction)
        state = get_state(robotId)
        terminal = False
        step = 0
        
        print(f"Episode {e+1} starting...")
        while not terminal and step < 800:
            state_tensor = torch.tensor(state, dtype=torch.float32, device=model.device)
            with torch.no_grad():
                q_values = model(state_tensor).detach().cpu().numpy()
            action_idx = np.argmax(q_values)
            
            send_action(robotId, action_idx)
            p.stepSimulation()
            time.sleep(1/60.) # Visual debugging speed
            
            state = get_state(robotId)
            
            if state[0] > 0.3 or state[0] < -0.3:
                print("  Failed: Hit rail boundary")
                terminal = True
            
            step += 1
            
        print(f"Episode {e+1} finished.")

    p.disconnect()

if __name__ == "__main__":
    test_model()
