import os
import pprint
import pygame
import threading

class LogitechController(object):
    """Class representing the PS4 controller. Pretty straightforward functionality."""

    controller = None
    axis_data = None

    def __init__(self):
        """Initialize the joystick components"""
        
        pygame.init()
        pygame.joystick.init()
        self.controller = pygame.joystick.Joystick(0)
        self.controller.init()
        self.axis_data = {
            0: 0,
            1: 0,
            2: 0,
            3: 0
        }

    def listen(self):
        """Listen for events to happen"""

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axis_data[event.axis] = round(event.value,2)

            pprint.pprint(self.axis_data)


def controller_main():
    logitech = LogitechController()
    logitech.listen()


if __name__ == "__main__":
    controller_main()