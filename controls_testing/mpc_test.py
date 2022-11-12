from matplotlib.patches import FancyArrow
import numpy as np
import time
import matplotlib.pyplot as plt
## Tasks
# [X] Non linear dynamics
# [X] Visualize control inputs
# [ ] Linearization function
# [ ] Use LQR to find control mat
# [ ] 

## Notes
# motor accel to speed?
L = 0.2
t_delta = 0.1 # sec
x = [0,0,0] # x, y, theta
u = [0.3,0.8] #steer, speed

def non_linear_dynamics(x, u):
    heading = x[2]
    steer = u[0]
    speed = u[1]
    x_dot = np.zeros_like(x)
    x_dot[0] = speed*np.cos(heading) # x_dot
    x_dot[1] = speed*np.sin(heading) # y_dot
    x_dot[2] = np.tan(steer)*speed/L # theta_dot
    return x_dot

plt.ion()
W = 2
fig = plt.figure()
ax = fig.add_subplot(111)
ax.axis("equal")
ax.set_xlim([-W, W])
ax.set_ylim([-W, W])
arrow : FancyArrow = ax.arrow(x[0], x[1], 0.5, 0.5, head_width=0.05, head_length=0.1, fc='k', ec='k')

while True:
    x_dot = non_linear_dynamics(x, u)
    x += x_dot*t_delta
    # print(x)
    wheel_base_center = x[0:2]
    front_center = x[0:2] + np.array([np.cos(x[2])*L, np.sin(x[2])*L])
    arrow.set_xy([wheel_base_center,front_center])
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(t_delta)