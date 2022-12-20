import copy
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
import numpy as np
import time
import matplotlib.pyplot as plt
from .geometry import AckermannState


class SimulatedAckermannDrive:
    """
    This simulated ackermann drive class possesses the full logical functionality
    of the AckermannDrive class, but abstracts out the hardware require to allow
    for more controlled testing. The hardware is replaced by a detailed visualization
    of the robot motion.
    """

    def __init__(self):
        """
        Constructor that instantiates physical constants and the visualization
        to track the simulated motion.
        """
        # Declare physical constants of the hardware
        self.WHEEL_BASE: float = 0.2  # m
        self.TRACK_WIDTH: float = 0.15  # m
        self.wheel_dim = np.array([0.1, 0.05])

        # Ackerman state and controller input
        self.state = np.array(
            [0, 0, 0, 0, 0.01]
        )  # x, y, theta, steer_angle, forward_speed
        self.u = np.zeros((2, 1))

        self.previous_odom_time = None
        self.previous_set_input_time = None

        # Set up the visualization with a car model and tracking of the path
        plt.ion()
        self.fig = plt.figure()
        self.fig.set_size_inches(10, 8, forward=True)
        self.ax = self.fig.add_subplot(111)
        self.ax.set_xlim([-0.5, 10])
        self.ax.set_ylim([-10, 10])
        self.ax.axis("equal")
        self.wheel_base_line = self.ax.add_line(Line2D([], []))
        self.back_track_width_line = self.ax.add_line(Line2D([], []))
        self.front_track_width_line = self.ax.add_line(Line2D([], []))
        wheel_rect = Rectangle(
            (0, 0),
            self.wheel_dim[0],
            self.wheel_dim[1],
            fc="none",
            ec="k",
            lw=1,
            rotation_point="center",
        )
        self.wheel_rects = [
            self.ax.add_patch(copy.copy(wheel_rect)),
            self.ax.add_patch(copy.copy(wheel_rect)),
            self.ax.add_patch(copy.copy(wheel_rect)),
            self.ax.add_patch(copy.copy(wheel_rect)),
        ]

    def set_steering_angle(self, phi: float):
        """
        Set the simulated steering angle as phi (rad).
        """
        self.state[3] = phi

    def set_drive_velocity(self, vel: float):
        """
        Set the simulated drive velocity as vel (m/s).
        """
        self.state[4] = vel

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
        if not self.previous_odom_time:
            self.previous_odom_time = time.time_ns()
            return
        cur_time = time.time_ns()
        t_delta = (cur_time - self.previous_odom_time) / (10**9)
        x_dot = self.non_linear_dynamics()
        self.state += x_dot * t_delta
        self.update_car_drawing()
        self.previous_odom_time = cur_time
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

    def non_linear_dynamics(self):
        """
        Determines the current instanteous change to the ackermann state (
        vel_x, vel_y, vel_heading, steer_angle_speed, accel_for)
        """
        x = self.state
        L = self.WHEEL_BASE
        x_dot = np.zeros_like(x)
        x_dot[0] = x[4] * np.cos(x[2])  # x_dot
        x_dot[1] = x[4] * np.sin(x[2])  # y_dot
        x_dot[2] = np.tan(x[3]) * x[4] / L  # theta_dot
        x_dot[3] = self.u[0]  # steer_angular_speed
        x_dot[4] = self.u[1]  # forward_accel
        return x_dot

    def get_linearized_system_matrix(self) -> np.ndarray:
        """
        Return the linearized ackermann drive system in matrix form
        """
        x = self.state
        L = self.WHEEL_BASE
        return np.array(
            [
                [0, 0, -np.sin(x[2]) * x[4], 0, np.cos(x[2])],
                [0, 0, np.cos(x[2]) * x[4], 0, np.sin(x[2])],
                [0, 0, 0, (1 / (np.cos(x[3]) ** 2)) * x[4] / L, np.tan(x[3]) / L],
                [0, 0, 0, 0, 0],
                [0, 0, 0, 0, 0],
            ]
        )

    def get_input_matrix(self) -> np.ndarray:
        """
        Return the default ackermann state matrix
        """
        return np.array(
            [[0, 0], [0, 0], [0, 0], [1, 0], [0, 1]]  # steering angle
        )  # forward accel

    def set_control_input(self, u):
        """
        Based on an external controls algorithm, the current control input (u)
        is set. This updates the control input aspect of the ackermann state and
        the drive angle and velocity are set accordingly.
        """
        # If no previous time, record the current time and return
        if not self.previous_set_input_time:
            self.previous_set_input_time = time.time_ns()
            return
        self.u = u
        current_time = time.time_ns()

        # Update the steering angle and forward speed based on the control input
        # and time difference between control inputs
        t_delta = (current_time - self.previous_set_input_time) / (10**9)
        self.state[3:] += u * t_delta

        # Set the steering angle and drive velocity to match the updated state
        self.set_steering_angle(self.state[3])
        self.set_drive_velocity(self.state[4])

        self.previous_set_input_time = current_time

    def update_car_drawing(self):
        """
        Update the visualization of the car model based on the new car state.
        """
        x = self.state
        L = self.WHEEL_BASE
        W = self.TRACK_WIDTH
        back_axle_center = x[0:2]
        front_axle_center = back_axle_center + [np.cos(x[2]) * L, np.sin(x[2]) * L]

        # Determine positions for each wheel center
        wheel_centers = [
            [
                back_axle_center[0] + np.sin(-x[2]) * W / 2,
                back_axle_center[1] + np.cos(-x[2]) * W / 2,
            ],
            [
                back_axle_center[0] - np.sin(-x[2]) * W / 2,
                back_axle_center[1] - np.cos(-x[2]) * W / 2,
            ],
            [
                front_axle_center[0] + np.sin(-x[2]) * W / 2,
                front_axle_center[1] + np.cos(-x[2]) * W / 2,
            ],
            [
                front_axle_center[0] - np.sin(-x[2]) * W / 2,
                front_axle_center[1] - np.cos(-x[2]) * W / 2,
            ],
        ]
        phi = x[3]
        phi_l = np.arctan(2 * L * np.sin(phi) / (2 * L * np.cos(phi) - W * np.sin(phi)))
        phi_r = np.arctan(2 * L * np.sin(phi) / (2 * L * np.cos(phi) + W * np.sin(phi)))
        wheel_angles = [x[2], x[2], x[2] + phi_l, x[2] + phi_r]

        # Update visualization location for each element of the car
        self.wheel_base_line.set_data(
            [[x[0], front_axle_center[0]], [x[1], front_axle_center[1]]]
        )
        self.back_track_width_line.set_data(
            [
                [wheel_centers[0][0], wheel_centers[1][0]],
                [wheel_centers[0][1], wheel_centers[1][1]],
            ]
        )
        self.front_track_width_line.set_data(
            [
                [wheel_centers[2][0], wheel_centers[3][0]],
                [wheel_centers[2][1], wheel_centers[3][1]],
            ]
        )

        for wheel, angle, center in zip(self.wheel_rects, wheel_angles, wheel_centers):
            wheel.set_xy(center - self.wheel_dim / 2)
            wheel.set_angle(np.degrees(angle))

    def curvature_to_steering(self, k):
        """
        Convert an inputted curvature (1/m) to steering angle (rad).
        """
        if k == 0:
            return 0
        r = 1 / k
        return np.arctan(self.WHEEL_BASE / r)
