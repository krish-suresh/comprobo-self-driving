from typing import List
from .geometry import Pose2D, AckermannState
from .drive import AckermannDrive
import numpy as np
import control

# TODO don't use AckermanState and make it abstract to state type from drive system

class MPCController:
    def __init__(self, drive, tolerance):
        self.drive : AckermannDrive = drive
        self.tolerance = tolerance

        # TODO Should prob live in an LQRController
        self.Q = np.eye(5)
        self.Q[0][0] = self.Q[1][1] = 100
        self.Q[2][2] = 100
        self.R = np.eye(2)
        self.R[0][0] = 20
        self.R[1][1] = 10

        self.waypoints : List[AckermannState] = []
        self.is_following : bool = False
        self.current_goal = None

    def update(self):
        # TODO add linearization delay
        if self.is_following:
            A = self.drive.get_linearized_system_matrix()
            B = self.drive.get_input_matrix()
            K = control.lqr(A, B, self.Q, self.R)[0]
            x = self.drive.state
            u = K @ (self.current_goal-x)
            self.drive.set_control_input(u)
            if np.linalg.norm(x[0:3]-self.current_goal[0:3]) < self.tolerance:
                if self.waypoints:
                    self.current_goal = self.waypoints.pop(0).to_vector()
                else:
                    self.is_following = False
        else:
            self.drive.set_control_input(np.zeros((2)))
            self.drive.set_drive_velocity(0)

    def follow_waypoints(self, waypoints : List[AckermannState]):
        if len(waypoints) < 1:
            raise ValueError("Must follow atleast 1 waypoint")
        self.is_following = True
        self.waypoints = waypoints
        self.current_goal = self.waypoints.pop(0).to_vector()