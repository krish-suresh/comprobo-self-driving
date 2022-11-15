from dataclasses import dataclass
import numpy as np

@dataclass
class Pose2D:
    x : float
    y : float
    theta : float

    def __init__(self, state_vector : np.ndarray):
        self.x, self.y, self.theta = state_vector

    def to_vector(self) -> np.ndarray:
        return np.array([self.x, self.y, self.theta])


#TODO prob should move somewhere else
@dataclass
class AckermannState:
    pose: Pose2D
    steer_angle : float
    drive_velocity : float

    def __init__(self, state):
        state_vector = np.array(state)
        self.pose = Pose2D(state_vector[0:3])
        self.steer_angle = state_vector[3]
        self.drive_velocity = state_vector[4]
    def to_vector(self) -> np.ndarray:
        return np.concatenate((self.pose.to_vector(),np.array([self.steer_angle, self.drive_velocity])))