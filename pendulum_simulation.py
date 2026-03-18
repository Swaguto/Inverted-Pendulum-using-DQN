import pybullet as p
import pybullet_data
import time
import os
import sys
import numpy as np

def run_simulation(urdf_path="robot/robot.urdf", use_fixed_base=True, start_pos=[0, 0, 1.0]):
    """
    Main simulation function for the Inverted Pendulum.
    """
    print(f"--- Inverted Pendulum Simulation ---")
    print(f"Python version: {sys.version.split()[0]}")
    print(f"PyBullet version: {p.getAPIVersion()}")
    
    # 1. Connect to PyBullet GUI
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # 2. Load Environment
    p.loadURDF("plane.urdf")
    
    # 3. Load Robot
    if not os.path.exists(urdf_path):
        print(f"Error: {urdf_path} not found. Ensure onshape-to-robot export is complete.")
        p.disconnect()
        return

    robotId = p.loadURDF(urdf_path, start_pos, useFixedBase=use_fixed_base)
    
    # 4. Inspect Joints
    numJoints = p.getNumJoints(robotId)
    print(f"\nDetected {numJoints} joints:")
    jointSliders = []
    
    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        jointIndex = info[0]
        jointName = info[1].decode("utf-8")
        jointType = info[2]
        lower_limit = info[8]
        upper_limit = info[9]
        
        print(f"  - Joint {i}: {jointName} (Type: {jointType})")
        
        # Setup sliders (Fixed for prismatic vs revolute)
        if lower_limit >= upper_limit:
            if jointType == p.JOINT_PRISMATIC:
                lower_limit, upper_limit = -0.5, 0.5
            else:
                lower_limit, upper_limit = -3.14, 3.14
        
        # Add slider to GUI
        slider = p.addUserDebugParameter(f"Target {jointName}", lower_limit, upper_limit, 0)
        jointSliders.append((jointIndex, slider, jointName))

    # Global Max Force Slider
    maxForceSlider = p.addUserDebugParameter("Max Motor Force (Set to 0 for free swing)", 0, 100, 50)

    # 5. Initialization: Tilt the pendulum so it falls
    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        if b"pendulum" in info[1]:
            p.resetJointState(robotId, i, targetValue=0.1)

    print("\nSimulation Ready.")
    print("TIP: Set 'Max Motor Force' to 0 to see the pendulum swing freely.")
    
    # 6. Main Loop
    try:
        while p.isConnected():
            maxForce = p.readUserDebugParameter(maxForceSlider)
            
            for jointIndex, sliderId, jointName in jointSliders:
                targetPos = p.readUserDebugParameter(sliderId)
                
                if "pendulum" in jointName:
                    # Pendulum is passive (free swing)
                    p.setJointMotorControl2(
                        bodyIndex=robotId,
                        jointIndex=jointIndex,
                        controlMode=p.VELOCITY_CONTROL,
                        targetVelocity=0,
                        force=0
                    )
                else:
                    # Cart is active (controlled by slider)
                    p.setJointMotorControl2(
                        bodyIndex=robotId,
                        jointIndex=jointIndex,
                        controlMode=p.POSITION_CONTROL,
                        targetPosition=targetPos,
                        force=maxForce
                    )

                # Observations
                jointState = p.getJointState(robotId, jointIndex)
                pos = jointState[0]
                if "pendulum" in jointName:
                    print(f"\r[OBS] Pole Angle: {pos:+.4f} rad ", end="")
            
            p.stepSimulation()
            time.sleep(1./240.)
            
    except KeyboardInterrupt:
        print("\nSimulation stopped by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    run_simulation()
