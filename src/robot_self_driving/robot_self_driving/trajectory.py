import math
import numpy as np
from .curves import CubicSpline2D

class TrapezoidalMotionProfile:
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

class CubicSplineTrajectory:
    def __init__(self, spline : CubicSpline2D, motion_profile : TrapezoidalMotionProfile):
        self.spline = spline
        self.motion_profile = motion_profile
    def state(self, t):
        s, v, a = self.motion_profile.state(t)
        x, y = self.spline.calc_position(s)
        theta = self.spline.calc_yaw(s)
        k = self.spline.calc_curvature(s)
        return np.array([x,y,theta,k,v])
