import numpy as np

curr_state = np.zeros((3,1))
curr_state[0,0], curr_state[1,0], curr_state[2,0] = 35.72439801, 103.8580862, 6.275890393

linear_vel, steer_angle = -10.49377735, 0.332874584

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