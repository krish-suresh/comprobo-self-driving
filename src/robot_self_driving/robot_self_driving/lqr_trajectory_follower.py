import math
import control
import numpy as np
import time
from .trajectory import CubicSplineTrajectory
from .drive import AckermannDrive

class AckermanLQRTrajectoryFollower:
    def __init__(self, drive : AckermannDrive, node):
        self.drive : AckermannDrive = drive
        self.trajectory : CubicSplineTrajectory = None

        self.Q = np.eye(5)
        self.Q[0][0] = self.Q[1][1] = 50
        self.Q[2][2] = 100
        self.R = np.eye(2)
        self.R[0][0] = 20
        self.R[1][1] = 15
        self.is_following : bool = False
        self.following_start_time = None
        self.logger = node.get_logger()

    def update(self):
        if self.is_following:
            t = time.time() - self.following_start_time
            current_goal = self.trajectory.state(t)
            current_goal[3] = self.drive.curvature_to_steering(current_goal[3])
            self.logger.info(f"X:{np.around(self.drive.state, 2)} T:{np.around(current_goal,2)}")
            # if np.all((self.drive.state == 0)):
            #     self.drive.state = np.array([0, 0, 0, 0, 0.01])
            # print(np.around(self.drive.state, 2))
            A = self.drive.get_linearized_system_matrix()
            B = self.drive.get_input_matrix()
            K = control.lqr(A, B, self.Q, self.R)[0]
            x = self.drive.state
            e = current_goal-x
            e[2] = (e[2] + np.pi) % (2 * np.pi) - np.pi
            u = K @ e
            self.drive.set_control_input(u)
            if t > self.trajectory.motion_profile.t_end:
                self.is_following = False
                self.drive.set_control_input(np.zeros((2)))
                self.drive.set_drive_velocity(0)

    def follow_trajectory(self, trajectory : CubicSplineTrajectory):
        self.is_following = True
        self.following_start_time = time.time()
        self.trajectory = trajectory