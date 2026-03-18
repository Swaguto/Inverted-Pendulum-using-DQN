"""
Diagnose environment: manually check how long the pole stays up with NO action.
This tells us if the physics are solvable.
"""
import pybullet as p
import numpy as np
from pendulum_env import InvertedPendulumEnv

env = InvertedPendulumEnv(render_mode=None)
obs, _ = env.reset()

print("Initial obs:", obs)
print("Format: [cart_pos_norm, cart_vel_norm, pole_angle, pole_vel_norm]")
print()

step = 0
for _ in range(1000):
    obs, reward, terminated, truncated, _ = env.step([0.0])  # Do nothing
    step += 1
    if step % 10 == 0 or terminated:
        real_angle_deg = np.degrees(obs[2])
        print(f"Step {step}: angle={real_angle_deg:.2f}deg, cart_pos={obs[0]:.3f}")
    if terminated:
        print(f"Terminated at step {step} — angle exceeded 45 deg OR cart out of bounds")
        break

env.close()
