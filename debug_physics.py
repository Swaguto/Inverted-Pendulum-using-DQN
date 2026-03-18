import pybullet as p
import pybullet_data
import time
import numpy as np

def debug_sim():
    print("Connecting to GUI...")
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    p.loadURDF("plane.urdf")
    
    print("Loading robot...")
    robotId = p.loadURDF("robot/robot.urdf", [0, 0, 1.0], useFixedBase=True)
    
    # Joint indices: 0 is slider, 1 is pivot
    cart_idx = 0
    pole_idx = 1
    
    print("Setting initial tilt...")
    p.resetJointState(robotId, pole_idx, targetValue=0.5) # 0.5 rad tilt
    p.setJointMotorControl2(robotId, pole_idx, p.VELOCITY_CONTROL, force=0) # Free swing
    
    print("Starting simulation loop (100 steps)...")
    for i in range(100):
        p.stepSimulation()
        pos = p.getJointState(robotId, pole_idx)[0]
        print(f"Step {i}: Pole Angle = {pos:.4f}")
        time.sleep(1./240.)
    
    print("Simulation test complete.")
    p.disconnect()

if __name__ == "__main__":
    debug_sim()
