import pigpio
import time
import numpy as np

conn = pigpio.pi()

MIN_SERVO_PWM = 50
MAX_SERVO_PWM = 2500
CENTER_SERVO_PWM = 1550
LEFT_SERVO_PWM = 2250
phi = np.radians(30)
RAD_TO_PWM = +900 / (np.pi / 2)
servo_pwm = phi * RAD_TO_PWM + CENTER_SERVO_PWM
print(servo_pwm)
# servo_pwm = CENTER_SERVO_PWM - 600
conn.set_servo_pulsewidth(14, servo_pwm)
