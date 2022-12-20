from dataclasses import dataclass
import numpy as np


@dataclass
class Pose2D:
    """
    Data class to store a two dimensional pose

    Attributes:
        x: horizontal position (m)
        y: vertical position (m)
        theta: current heading (rad)
    """

    x: float
    y: float
    theta: float

    def __init__(self, state_vector: np.ndarray):
        self.x, self.y, self.theta = state_vector

    def to_vector(self) -> np.ndarray:
        """
        Return the current pose as a 1D numpy array.
        """
        return np.array([self.x, self.y, self.theta])


@dataclass
class AckermannState:
    """
    Data class to store ackermann states (x pos, y pos, heading, steering angle,
    forward velocity)

    Attributes:
        pose: pose2D object
        steer_angle: Current robot steering angle (rad)
        drive_velocity: Current robot forward speed (m/s)
    """

    pose: Pose2D
    steer_angle: float
    drive_velocity: float

    def __init__(self, state):
        state_vector = np.array(state)
        self.pose = Pose2D(state_vector[0:3])
        self.steer_angle = state_vector[3]
        self.drive_velocity = state_vector[4]

    def to_vector(self) -> np.ndarray:
        """
        Return the current ackermann state as a 1D numpy array
        """
        return np.concatenate(
            (self.pose.to_vector(), np.array([self.steer_angle, self.drive_velocity]))
        )
