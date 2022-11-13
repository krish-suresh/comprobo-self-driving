import copy
from typing import List
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow, Rectangle
import numpy as np
import time
import matplotlib.pyplot as plt
import control
# Tasks
# [X] Non linear dynamics
# [X] Visualize control inputs
# [X] Linearization function
# [X] Use LQR to find control mat
# [X] Draw wheels and wheel base and target poses
# [X] Add target pose and test lqr
# [X] Implement MPC controls
# [ ] Tune LQR and add linearization buffer
# [ ] Attempt discrete model
# Notes

# Constants
L = 0.2
W = 0.15
wheel_dim = np.array([0.1, 0.05])
t_delta = 0.01  # sec
x = np.array([0, 0, 0, 0, 0.01])  # x, y, theta, steer_angle, forward_speed
u = [0, 0]  # steer_angle_speed, forward_accel
B = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [1, 0],  # steering angle
              [0, 1]])  # forward accel
Q = np.eye(5)
Q[0][0] = Q[1][1] = 100
Q[2][2] = 10
R = np.eye(2)
R[0][0] = 2
R[1][1] = 0.1
# Functions


def non_linear_dynamics(x, u):
    x_dot = np.zeros_like(x)
    x_dot[0] = x[4]*np.cos(x[2])  # x_dot
    x_dot[1] = x[4]*np.sin(x[2])  # y_dot
    x_dot[2] = np.tan(x[3])*x[4]/L  # theta_dot
    x_dot[3] = u[0]  # steer_angular_speed
    x_dot[4] = u[1]  # forward_accel
    return x_dot


def linearization(x):
    A = np.array([[0, 0, -np.sin(x[2])*x[4], 0, np.cos(x[2])],
                  [0, 0, np.cos(x[2])*x[4], 0, np.sin(x[2])],
                  [0, 0, 0, (1/(np.cos(x[3])**2))*x[4]/L, np.tan(x[3])/L],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0]])
    return A


def update_car_drawing(x, wheel_base, back_track_width, front_track_width, wheels: List[Rectangle]):
    back_axle_center = x[0:2]
    front_axle_center = back_axle_center + [np.cos(x[2])*L, np.sin(x[2])*L]

    wheel_centers = [[back_axle_center[0] + np.sin(-x[2])*W/2, back_axle_center[1] + np.cos(-x[2])*W/2],
                     [back_axle_center[0] -
                         np.sin(-x[2])*W/2, back_axle_center[1] - np.cos(-x[2])*W/2],
                     [front_axle_center[0] +
                         np.sin(-x[2])*W/2, front_axle_center[1] + np.cos(-x[2])*W/2],
                     [front_axle_center[0] - np.sin(-x[2])*W/2, front_axle_center[1] - np.cos(-x[2])*W/2]]
    phi = x[3]
    phi_l = np.arctan(2*L*np.sin(phi)/(2*L*np.cos(phi) - W*np.sin(phi)))
    phi_r = np.arctan(2*L*np.sin(phi)/(2*L*np.cos(phi) + W*np.sin(phi)))
    wheel_angles = [x[2], x[2], x[2]+phi_l, x[2]+phi_r]
    wheel_base.set_data([[x[0], front_axle_center[0]],
                        [x[1], front_axle_center[1]]])
    back_track_width.set_data([[wheel_centers[0][0], wheel_centers[1][0]], [
                              wheel_centers[0][1], wheel_centers[1][1]]])
    front_track_width.set_data([[wheel_centers[2][0], wheel_centers[3][0]], [
                               wheel_centers[2][1], wheel_centers[3][1]]])

    for wheel, angle, center in zip(wheels, wheel_angles, wheel_centers):
        wheel.set_xy(center-wheel_dim/2)
        wheel.set_angle(np.degrees(angle))


# Controls Setup
r = np.array([0.5, 0, 0, 0, 0])
# A = linearization(x)
# K = control.lqr(A, B, Q, R)[0]
# print(np.linalg.eig(A-np.matmul(B,K))[0])
# print(K)
# quit()

# Plot Setup
plt.ion()
plot_w = 1.5
# fig = plt.figure()
# ax = fig.add_subplot(111)
fig, axs = plt.subplots(2, 2)
fig.set_size_inches(18.5, 10.5, forward=True)

gs = axs[0, 0].get_gridspec()
axs[0, 0].remove()
axs[0, 1].remove()
ax = fig.add_subplot(gs[0, :])

ax.axis("equal")
ax.set_xlim([-plot_w, plot_w])
ax.set_ylim([-plot_w, plot_w])
wheel_base = ax.add_line(Line2D([], []))
back_track_width = ax.add_line(Line2D([], []))
front_track_width = ax.add_line(Line2D([], []))
wheel_rect = Rectangle(
    (0, 0), wheel_dim[0], wheel_dim[1], fc='none', ec='k', lw=1, rotation_point="center")
wheels = [ax.add_patch(copy.copy(wheel_rect)),
          ax.add_patch(copy.copy(wheel_rect)),
          ax.add_patch(copy.copy(wheel_rect)),
          ax.add_patch(copy.copy(wheel_rect)), ]
ax.plot(r[0], r[1], marker="o", markersize=2)

padding = 0.25
ts = [0]
u_hist = np.array([u])
u_int_hist = np.array([[0,0]])
steer_speed_line, = axs[1, 0].plot(ts, u_hist[:, 0])
steer_angle_line, = axs[1, 0].plot(ts, u_int_hist[:, 0])
axs[1, 0].set_xlim([0, ts[-1]+padding])
axs[1, 0].set_ylim([-5, 5])
axs[1, 0].set_xlabel("time (s)")
# axs[1,0].set_ylabel("steering angular speed (rad/s)")

accel_line, = axs[1, 1].plot(ts, u_hist[:, 1])
vel_line, = axs[1, 1].plot(ts, u_int_hist[:, 1])
axs[1, 1].set_xlim([0, ts[-1]+padding])
axs[1, 1].set_ylim([-5, 5])
axs[1, 0].set_xlabel("time (s)")
# axs[1,1].set_ylabel("forward acceleration (m/s^2)")

# Simulation Loop
while True:
    A = linearization(x)
    K = control.lqr(A, B, Q, R)[0]
    u = K @ (r-x)
    x_dot = non_linear_dynamics(x, u)
    x += x_dot*t_delta

    update_car_drawing(x, wheel_base, back_track_width,
                       front_track_width, wheels)
    steer_speed_line.set_xdata(ts)
    steer_speed_line.set_ydata(u_hist[:, 0])
    accel_line.set_xdata(ts)
    accel_line.set_ydata(u_hist[:, 1])
    steer_angle_line.set_xdata(ts)
    steer_angle_line.set_ydata(u_int_hist[:, 0])
    vel_line.set_xdata(ts)
    vel_line.set_ydata(u_int_hist[:, 1])
    axs[1, 0].set_xlim([0, ts[-1]+padding])
    axs[1, 0].set_ylim([min(u_hist[:, 0])-padding, max(u_hist[:, 0])+padding])
    axs[1, 1].set_xlim([0, ts[-1]+padding])
    axs[1, 1].set_ylim([min(u_hist[:, 1])-padding, max(u_hist[:, 1])+padding])
    fig.canvas.draw()
    fig.canvas.flush_events()
    ts.append(ts[-1]+t_delta)
    u_hist = np.vstack([u_hist, u])
    u_int = u_int_hist[-1] + t_delta*u
    u_int_hist = np.vstack([u_int_hist, u_int])
    time.sleep(t_delta)
