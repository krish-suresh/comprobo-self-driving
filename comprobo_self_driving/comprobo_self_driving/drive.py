import RPi.GPIO as GPIO
from gpiozero import Servo
import pigpio
import time
import numpy as np 
from geometry import AckermannState

SERVO_PIN: int = 0
ENCODER_PIN: int = 0

class AckermannDrive():
    """
    """

    def __init__(self, pin):
        """
        """
        self.servo = Servo(SERVO_PIN)   # Create Servo with gpio pin
        self.connect = pigpio.pi()
        self.pin = pin
        self.wheel_base = 0.2 # m
        self.MIN_WIDTH_ESC: int = 0
        self.MAX_WIDTH_ESC: int = 0
        

    def set_steering_angle(self, theta):
        """
        """
        val = theta*10
        self.servo.value = val
        print(val)


    def set_drive_velocity(self, width, snooze):
        """
        """
        self.pwm(width=width, snooze=snooze)

    def pwm(self, width: int, snooze: int = 0):
        """
        """
        self.connect.set_servo_pulsewidth(self.pin, width)
        if snooze:
            time.sleep(snooze)
        return

    def arm_esc(self):
        """
        """
        self.pwm(width=self.MIN_WIDTH_ESC, snooze = 4)

    def calibrate_esc(self):
        """
        """
        self.pwm(width=self.MAX_WIDTH_ESC)
        self.pwm(width=self.MAX_WIDTH_ESC, snooze=2)
        self.pwm(width=self.MIN_WIDTH_ESC, snooze=4)

    def get_state(self) -> AckermannState:
        """
        Return Ackermann state
        """
        return AckermannState

    def update(self):
        """
        update odom
        """
        pass


    def get_linearized_system_matrix(self) -> np.ndarray:
        x = self.get_state().to_vector()
        L = self.wheel_base
        return np.array([[0, 0, -np.sin(x[2])*x[4], 0, np.cos(x[2])],
                  [0, 0, np.cos(x[2])*x[4], 0, np.sin(x[2])],
                  [0, 0, 0, (1/(np.cos(x[3])**2))*x[4]/L, np.tan(x[3])/L],
                  [0, 0, 0, 0, 0],
                  [0, 0, 0, 0, 0]])

    def get_input_matrix(self) -> np.ndarray:
        return np.array([[0, 0],
                         [0, 0],
                         [0, 0],
                         [1, 0],  # steering angle
                         [0, 1]])  # forward accel

    def set_control_input(self, u):
        self.set_steering_angle(u[0])
        self.set_drive_velocity(u[1])