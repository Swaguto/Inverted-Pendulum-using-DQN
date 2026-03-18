import pybullet as p
import numpy as np

def inspect_axes():
    p.connect(p.DIRECT)
    robotId = p.loadURDF("robot/robot.urdf", [0, 0, 1.0], useFixedBase=True)
    
    for i in range(p.getNumJoints(robotId)):
        info = p.getJointInfo(robotId, i)
        name = info[1].decode()
        axis_local = info[13]
        
        # Get link state to transform axis
        state = p.getLinkState(robotId, i)
        # Link orientation (world)
        link_ori = state[1] 
        
        # Transform local axis to world axis
        axis_world, _ = p.multiplyTransforms([0,0,0], link_ori, axis_local, [0,0,0,1])
        
        # Get link world pos to see if it's pointing up
        link_world_pos = state[0]
        parent_world_pos = p.getLinkState(robotId, info[16])[0] if info[16] != -1 else p.getBasePositionAndOrientation(robotId)[0]
        
        vec = np.array(link_world_pos) - np.array(parent_world_pos)
        
        print(f"Joint {i} ({name}):")
        print(f"  World Axis: {axis_world}")
        print(f"  Link Index: {info[16]} (Parent - Link vector: {vec})")
        print(f"  Link World Pos: {link_world_pos}")
        
    p.disconnect()

if __name__ == "__main__":
    inspect_axes()
