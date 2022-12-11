import math
import control
import numpy as np
import time
import matplotlib.pyplot as plt
from curves import CubicSpline2D, RaceTrack
from trajectory import TrapezoidalMotionProfile, CubicSplineTrajectory, RotationLimitedMotionProfile
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow, Rectangle
from typing import List
import copy
class AckermanLQRTrajectoryFollower:
    def __init__(self, trajectory : CubicSplineTrajectory):
        self.trajectory = trajectory


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

def curvature_to_steering(k):
    if k == 0:
        return 0
    r = 1/k
    return math.atan(L/r)


def test_motion_profile():
    mp = TrapezoidalMotionProfile(2,2,1)        
    t = np.linspace(0,mp.t_end,200)
    s = []
    v = []
    a = []
    for t_i in t:
        si,vi,ai = mp.state(t_i)
        s.append(si)
        v.append(vi)
        a.append(ai)
    plt.plot(t,s)
    plt.plot(t,v)
    plt.plot(t,a)
    plt.show()

def test_spline_trajectory():
    plt.ion()
    fig = plt.figure()
    ax = fig.add_subplot(111)
    xp = [0, 0.5, 1]
    yp = [0, 0, 1]
    sp = CubicSpline2D(xp, yp)
    mp = TrapezoidalMotionProfile(sp.s[-1],2,1)        
    trajectory = CubicSplineTrajectory(sp, mp)
    dt = 0.01
    t = np.arange(0,mp.t_end,dt)
    x = []
    y = []
    path_line, = ax.plot(x,y)
    target_point, = ax.plot(xp[0], yp[0], "xb")
    ax.plot(xp, yp, "xr")
    for t_i in t[:-1]:
        xi, yi, thetai, ki, vi = trajectory.state(t_i)
        x.append(xi)
        y.append(yi)
        path_line.set_xdata(x)
        path_line.set_ydata(y)
        target_point.set_xdata(xi)
        target_point.set_ydata(yi)
        fig.canvas.draw()
        fig.canvas.flush_events()
        time.sleep(dt)

L = 0.2
W = 0.15
wheel_dim = np.array([0.1, 0.05])
t_delta = 0.005  # sec
B = np.array([[0, 0],
              [0, 0],
              [0, 0],
              [1, 0],  # steering angle
              [0, 1]])  # forward accel
Q = np.eye(5)
Q[0][0] = Q[1][1] = 400
Q[2][2] = 100
Q[3][3] = 10
Q[4][4] = 1
R = np.eye(2)
R[0][0] = 2
R[1][1] = 0.005

x = np.array([0, 0, -np.pi/2, 0, 0.5])  # x, y, theta, steer_angle, forward_speed
u = [0, 0]  # steer_angle_speed, forward_accel

# sp = CubicSpline2D(xp, yp)
track = RaceTrack("./tracks/IMS_raceline.csv", 12.5, 7)
sp = track.create_spline()
xp = track.x
yp = track.y
# mp = TrapezoidalMotionProfile(sp.s[-1],2,1)        
mp = RotationLimitedMotionProfile(sp,2,5,3,0.01)
trajectory = CubicSplineTrajectory(sp, mp)
plt.ion()
fig = plt.figure()
fig.set_size_inches(10, 8, forward=True)
ax = fig.add_subplot(111)
ax.axis("equal")
ax.set_xlim([-0.5, 5])
ax.set_ylim([-5, 5])
dt = 0.01
t = np.arange(0,mp.t_end,dt)
xs = []
ys = []
xf = []
yf = []
path_line, = ax.plot(xs,ys)
follow_line, = ax.plot(0,0)
target_point, = ax.plot(0, 0, "xb")
ax.plot(xp, yp, "r")

wheel_base = ax.add_line(Line2D([], []))
back_track_width = ax.add_line(Line2D([], []))
front_track_width = ax.add_line(Line2D([], []))
wheel_rect = Rectangle(
    (0, 0), wheel_dim[0], wheel_dim[1], fc='none', ec='k', lw=1, rotation_point="center")
wheels = [ax.add_patch(copy.copy(wheel_rect)),
        ax.add_patch(copy.copy(wheel_rect)),
        ax.add_patch(copy.copy(wheel_rect)),
        ax.add_patch(copy.copy(wheel_rect)), ]


for t_i in t[:-1]:
    print(t_i)
    target = trajectory.state(t_i)
    target[3] = curvature_to_steering(target[3])
    xi, yi, thetai, ki, vi = trajectory.state(t_i)
    A = linearization(x)
    K = control.lqr(A, B, Q, R)[0]
    u = K @ (target-x)
    x_dot = non_linear_dynamics(x, u)
    x += x_dot*t_delta
    update_car_drawing(x, wheel_base, back_track_width,
                    front_track_width, wheels)
    path_line.set_xdata(xs)
    path_line.set_ydata(ys)
    follow_line.set_xdata(xf)
    follow_line.set_ydata(yf)
    target_point.set_xdata(xi)
    target_point.set_ydata(yi)
    xs.append(xi)
    ys.append(yi)
    xf.append(x[0])
    yf.append(x[1])
    fig.canvas.draw()
    fig.canvas.flush_events()
    time.sleep(dt)

