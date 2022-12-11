import math
import numpy as np
from .curves import CubicSpline2D
from abc import ABC, abstractmethod


class MotionProfile(ABC):
    @abstractmethod
    def state(self, t):
        pass

class TrapezoidalMotionProfile(MotionProfile):
    #      1          2
    #      ___________
    #     /           \ 
    #    /             \
    #   /               \
    #  /                 \
    # init                end
    def __init__(self, path_length, max_velocity, max_acceleration):
        self.path_length = path_length
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        if path_length < max_velocity**2/max_acceleration:
            self.t_1 = self.t_2 = math.sqrt(path_length/(max_acceleration))
            self.t_end = self.t_1*2
        else :  
            self.t_1 = max_velocity/max_acceleration
            self.t_2 = self.t_1 + (path_length - self.t_1*max_velocity)/max_velocity
            self.t_end = self.t_1 + self.t_2

    def state(self, t):
        if (t < 0):
            return 0, 0, 0
        if (t < self.t_1):
            return (t**2)*self.max_acceleration/2, t*self.max_acceleration, self.max_acceleration
        if (t < self.t_2):
            return (self.t_1**2)*self.max_acceleration/2 + (t-self.t_1)*self.max_velocity, self.max_velocity, 0
        if (t <= self.t_end):
            return (self.t_1**2)*self.max_acceleration/2 + (self.t_2-self.t_1)*self.max_velocity + (self.t_1**2)*self.max_acceleration/2 - ((self.t_end-t)**2)*self.max_acceleration/2, (self.t_end-t)*self.max_acceleration, -self.max_acceleration
        return 0,0,0



class RotationLimitedMotionProfile(MotionProfile):
    def __init__(self, path : CubicSpline2D, max_velocity, max_acceleration, max_angular_velocity, dt):
        self.path : CubicSpline2D = path
        self.max_velocity = max_velocity
        self.max_acceleration = max_acceleration
        self.max_angular_velocity = max_angular_velocity
        self.motion_profile = []

        s = 0
        t = 0
        v = 0
        a = 0
        while s <= path.s[-1]:
            print(v)
            self.motion_profile.append([s,v,a])
            k = path.calc_curvature(s)
            limited_v = max_velocity
            if k != 0:
                limited_v = min(max_velocity, abs(max_angular_velocity/k))
            if v < limited_v:
                v += max_acceleration*dt
                a = max_acceleration
            elif v > limited_v:
                v += -max_acceleration*dt
                a = -max_acceleration
            else:
                a = 0

            s += v*dt
            t += dt
        self.t_end = t
        self.steps = len(self.motion_profile)

    def state(self, t):
        if t>self.t_end:
            return 0,0,0
        return self.motion_profile[int((t/self.t_end)*self.steps)]

class CubicSplineTrajectory:
    def __init__(self, spline : CubicSpline2D, motion_profile : MotionProfile):
        self.spline = spline
        self.motion_profile = motion_profile
    def state(self, t):
        s, v, a = self.motion_profile.state(t)
        x, y = self.spline.calc_position(s)
        theta = self.spline.calc_yaw(s)
        k = self.spline.calc_curvature(s)
        return np.array([x,y,theta,k,v])
