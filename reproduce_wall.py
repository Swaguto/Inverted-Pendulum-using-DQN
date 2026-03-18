import pybullet as p
import pybullet_data
import time
import numpy as np

def reproduce_invisible_wall():
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
    
    # Get joint info to check limits
    info = p.getJointInfo(robotId, pole_idx)
    lower_limit = info[8]
    upper_limit = info[9]
    print(f"Joint 1 (Pole) Limits: {lower_limit} to {upper_limit}")
    
    # Give it a lot of initial tilt to make it swing hard
    p.resetJointState(robotId, pole_idx, targetValue=3.0) 
    p.setJointMotorControl2(robotId, pole_idx, p.VELOCITY_CONTROL, force=0) # Free swing
    
    print("Starting simulation loop... Watch for 'stuck' behavior.")
    # Run for 1000 steps to see more swings
    for i in range(1000):
        p.stepSimulation()
        pos, vel, _, _ = p.getJointState(robotId, pole_idx)
        print(f"\rStep {i:4d}: Pole Angle = {pos:+.4f}, Velocity = {vel:+.4f}", end="")
        if abs(pos - lower_limit) < 1e-3 or abs(pos - upper_limit) < 1e-3:
            print(f"\n[!!!] Hit Joint Limit at Step {i}!")
        time.sleep(1./240.)
    
    print("\nSimulation complete. Press Enter to exit.")
    input()
    p.disconnect()

if __name__ == "__main__":
    reproduce_invisible_wall()
