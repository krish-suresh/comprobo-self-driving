import pigpio
import time
import numpy as np 
from .geometry import AckermannState

class AckermannDrive():
    """
    """

    def __init__(self):
        """
        """
        self.connect = pigpio.pi()
        self.ESC_PIN: int = 15
        self.SERVO_PIN: int = 14
        self.WHEEL_BASE: float = 0.2 # m
        self.MIN_WIDTH_ESC: int = 1000
        self.MAX_WIDTH_ESC: int = 2000
        self.TIRE_DIAMETER: int = 0.1194      # mm
        self.GEAR_RATIO: int = 42
        self.MAX_RAW_RPM: int = 10400
        self.MAX_SERVO_PWM: int= 2500       # micro sec
        self.MIN_SERVO_PWM: int = 500
        self.MAX_VEL: float = ((self.MAX_RAW_RPM / self.GEAR_RATIO)/60) * ((self.TIRE_DIAMETER * np.pi))  # m/s
        self.u_integrated = np.zeros((2,1))
        self.state = np.array([0,0,0,0,0])
        self.u = np.zeros((2,1))

    def set_steering_angle(self, theta: float, snooze: int):
        """
        """
        print(f"Steering Angle: ", theta)
        steering_pwm_signal = (1.8 - theta)/np.pi * (self.MAX_SERVO_PWM - self.MIN_SERVO_PWM) + (self.MIN_SERVO_PWM)
        print(f"steering {steering_pwm_signal}")
        self.servo_pwm(width=steering_pwm_signal)

    def set_drive_velocity(self, vel: float, snooze: int):
        """
        """
        input_vel = vel / self.MAX_VEL
        print(f"Drive Velocity: ", input_vel)
        print(self.MAX_VEL)
        esc_pwm_signal = -input_vel*(self.MAX_WIDTH_ESC-self.MIN_WIDTH_ESC)/2 + self.MIN_WIDTH_ESC + (self.MAX_WIDTH_ESC-self.MIN_WIDTH_ESC)/2
        print(f"esc {esc_pwm_signal}")
        self.esc_pwm(width=esc_pwm_signal)

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
        self.esc_pwm(width=self.MIN_WIDTH_ESC)
        input("Connect power and press Enter to continue...")
        self.esc_pwm(width=self.MIN_WIDTH_ESC, snooze = 4)

    def calibrate_esc(self):
        """
        """
        self.esc_pwm(width=self.MAX_WIDTH_ESC)
        input("Connect power and press Enter to continue...")
        self.esc_pwm(width=self.MAX_WIDTH_ESC, snooze=2)
        self.esc_pwm(width=self.MIN_WIDTH_ESC, snooze=4)

    def get_state(self) -> AckermannState:
        """
        Return Ackermann state
        """
        return AckermannState(self.state)

    def update(self):
        """
        update odom
        """
        # TODO replace with encoder odom
        cur_time = time.time_ns()
        t_delta = (cur_time - self.previous_odom_time) / (10 ** 9)
        x_dot = self.non_linear_dynamics()
        self.state += x_dot*t_delta
        self.previous_odom_time = cur_time

    def non_linear_dynamics(self):
        x = self.get_state().to_vector()
        L = self.WHEEL_BASE
        x_dot = np.zeros_like(x)
        x_dot[0] = x[4]*np.cos(x[2])  # x_dot
        x_dot[1] = x[4]*np.sin(x[2])  # y_dot
        x_dot[2] = np.tan(x[3])*x[4]/L  # theta_dot
        x_dot[3] = self.u[0]  # steer_angular_speed
        x_dot[4] = self.u[1]  # forward_accel
        return x_dot

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
        self.u = u
        current_time = time.time_ns()
        t_delta = (current_time - self.previous_set_input_time)/ (10 ** 9)
        self.u_integrated += u*t_delta
        self.set_steering_angle(self.u_integrated[0])
        self.set_drive_velocity(self.u_integrated[1])
        self.previous_set_input_time = current_time
