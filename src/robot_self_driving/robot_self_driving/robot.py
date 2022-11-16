from .drive import AckermannDrive
from .mpc_controller import MPCController
from .simulated_ackermann_drive import SimulatedAckermannDrive

class Robot():
    """
    """

    def __init__(self, use_sim = False):
        """
        """
        self.drive = AckermannDrive() if not use_sim else SimulatedAckermannDrive()
        self.controller = MPCController(self.drive, 0.2) # TODO move tol somewhere else

    def set_steering_angle(self, theta: float):
        """
        """
        self.drive.set_steering_angle(theta)

    def set_drive_velocity(self, vel: float):
        """
        """
        self.drive.set_drive_velocity(vel)
    
    def update(self):
        """
        """
        self.drive.update()
        self.controller.update()
