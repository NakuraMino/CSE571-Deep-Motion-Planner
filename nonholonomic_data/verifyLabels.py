import numpy as np

curr_state = np.zeros((3,1))
curr_state[0,0], curr_state[1,0], curr_state[2,0] = 23.14042914, 58.30688408, 4.550857749

linear_vel, steer_angle = -13.00024525, -0.347909538

dt = 0.1
delta_step = 5

x = curr_state
L = 7.5 

rollout = [x]
for i in range(delta_step):
    x_rollout = np.zeros_like(x)
    x_rollout[0] = x[0] + linear_vel * np.cos(x[2]) * dt
    x_rollout[1] = x[1] + linear_vel * np.sin(x[2]) * dt
    x_rollout[2] = x[2] + (linear_vel/L) * np.tan(steer_angle) * dt
    x_rollout[2] = x_rollout[2] % (2*np.pi)

    x = x_rollout
    rollout.append(x_rollout) # maintain history

print(x)