import RPi.GPIO as GPIO
from gpiozero import Servo
import pigpio
import time

SERVO_PIN: int = 0
ENCODER_PIN: int

class AckermannDrive():
    """
    """

    def __init__(self, pin):
        """
        """
        self.servo = Servo(SERVO_PIN)   # Create Servo with gpio pin
        self.connect = pigpio.pi()
        self.pin = pin

        

    def set_steering_angle(self, theta):
        """
        """
        val = theta*10
        self.servo.value = val
        print(val)


    def set_drive_velocity(self, p):
        """
        """

    def pwm(self, width: int, snooze: int = 0):
        """
        """
        self.conn.set_servo_pulsewidth(self.pin, width)
        if snooze:
            time.sleep(snooze)
        return

    def arm_esc(self):
        """
        """
        self.pwm(width=self.MIN_WIDTH, snooze = 4)

    def get_state(self):
        """
        """
        pass

    def update(self):
        """
        """
        pass