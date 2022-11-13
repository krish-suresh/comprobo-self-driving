from drive import AckermannDrive
from mpc_controller import MPCController

class Robot():

    def __init__(self):
        self.drive = AckermannDrive()
        self.controller = MPCController(self.drive, 0.1) # TODO move tol somewhere else
    
    def update(self):
        self.drive.update()
        self.controller.update()
