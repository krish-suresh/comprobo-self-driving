from matplotlib.patches import FancyArrow
import numpy as np
import time
import matplotlib.pyplot as plt
import control
## Tasks
# [X] Non linear dynamics
# [X] Visualize control inputs
# [X] Linearization function
# [X] Use LQR to find control mat
# [ ] Draw wheels and wheel base and target poses
# [ ] Add target pose and test lqr
# [ ] Implement MPC controls 
## Notes
# motor accel to speed?
L = 0.2
t_delta = 0.1 # sec
x = [0,0,0,0.4,1] # x, y, theta, steer_angle, forward_speed
u = [0,0] # steer_angle_speed, forward_accel
B = np.array([[0,0],
              [0,0],
              [0,0],
              [1,0],  # steering angle
              [0,1]]) # forward accel
Q = np.eye(5)
R = np.eye(2)
def non_linear_dynamics(x, u):
    x_dot = np.zeros_like(x)
    x_dot[0] = x[4]*np.cos(x[2]) # x_dot
    x_dot[1] = x[4]*np.sin(x[2]) # y_dot
    x_dot[2] = np.tan(x[3])*x[4]/L # theta_dot
    x_dot[3] = u[0] # steer_angular_speed
    x_dot[4] = u[1] # forward_accel 
    return x_dot

def linearization(x): 
    A = np.array([[0, 0, -np.sin(x[2])*x[4], 0, np.cos(x[2])],
                  [0, 0, np.cos(x[2])*x[4], 0, np.sin(x[2])],
                  [0, 0, 0, (1/(np.cos(x[3])**2))*x[4]/L ,np.tan(x[3])/L],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0]])
    return A
A = linearization(x)
K = control.lqr(A,B,Q,R)[0]
print(np.linalg.eig(A-np.matmul(B,K))[0])
print(K)
quit()

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