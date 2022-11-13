import pigpio
import time
import numpy as np 
from geometry import AckermannState

class AckermannDrive():
    """
    """

    def __init__(self):
        """
        """
        self.connect = pigpio.pi()
        self.ESC_PIN: int = 0
        self.SERVO_PIN: int = 0
        self.WHEEL_BASE: float = 0.2 # m
        self.MIN_WIDTH_ESC: int = 1000
        self.MAX_WIDTH_ESC: int = 2000
        self.TIRE_DIAMETER: int = 119.4      # mm
        self.GEAR_RATIO: int = 42
        self.MAX_RAW_RPM: int = 10400
        self.MAX_SERVO_PWM: int= 2500       # micro sec
        self.MIN_SERVO_PWM: int = 500
        self.MAX_VEL: int = (self.MAX_RAW_RPM / self.GEAR_RATIO) * ((2 * self.TIRE_DIAMETER * np.pi) / 60)

    def set_steering_angle(self, theta: int, snooze: int):
        """
        """
        input_theta = theta/180 * (self.MAX_SERVO_PWM - self.MIN_SERVO_PWM) + (self.MIN_SERVO_PWM)
        input_radians = np.radians(input_theta)
        self.servo_pwm(width=input_radians, snooze=snooze)

    def set_drive_velocity(self, vel: float, snooze: int):
        """
        """
        input_vel = vel / self.MAX_VEL
        self.esc_pwm(width=input_vel, snooze=snooze)

    def esc_pwm(self, width: int, snooze: int = 0):
        """
        """
        self.connect.set_servo_pulsewidth(self.ESC_PIN, width)
        if snooze:
            time.sleep(snooze)
        return
    
    def servo_pwm(self, width: int, snooze: int = 0):
        """
        """
        self.connect.set_servo_pulsewidth(self.SERVO_PIN, width)
        if snooze:
            time.sleep(snooze)
        return

    def arm_esc(self):
        """
        """
        self.esc_pwm(width=self.MIN_WIDTH_ESC, snooze = 4)

    def calibrate_esc(self):
        """
        """
        self.esc_pwm(width=self.MAX_WIDTH_ESC)
        self.esc_pwm(width=self.MAX_WIDTH_ESC, snooze=2)
        self.esc_pwm(width=self.MIN_WIDTH_ESC, snooze=4)

    def get_state(self) -> AckermannState:
        """
        Return Ackermann state
        """
        pass 

    def update(self):
        """
        update odom
        """
        pass


    def get_linearized_system_matrix(self) -> np.ndarray:
        x = self.get_state().to_vector()
        L = self.WHEEL_BASE
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
        