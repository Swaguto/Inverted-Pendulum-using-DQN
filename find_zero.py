import pybullet as p
import numpy as np
import time

p.connect(p.DIRECT)
robot = p.loadURDF("robot/robot.urdf", [0,0,1], useFixedBase=True)

best_angle = 0
min_x_dist = 100

for angle in np.linspace(-1, 1, 2000):
    p.resetJointState(robot, 1, angle)
    p.stepSimulation()
    # Get world position of the pendulum link
    state = p.getLinkState(robot, 1)
    world_pos = state[0]
    parent_state = p.getLinkState(robot, 0)
    parent_pos = parent_state[0]
    
    # Calculate X distance from parent (cart)
    x_dist = abs(world_pos[0] - parent_pos[0])
    if x_dist < min_x_dist:
        min_x_dist = x_dist
        best_angle = angle

print(f"Best Angle for Vertical: {best_angle} rad ({np.degrees(best_angle)} deg)")
p.disconnect()
