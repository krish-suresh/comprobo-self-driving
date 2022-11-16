import pigpio
import time
import numpy as np
conn = pigpio.pi()

MIN_SERVO_PWM = 50
MAX_SERVO_PWM = 2500
CENTER_SERVO_PWM = 1250
LEFT_SERVO_PWM = 2250
phi = np.radians(-47)
RAD_TO_PWM = -1000/(np.pi/2)
servo_pwm = phi*RAD_TO_PWM + CENTER_SERVO_PWM
conn.set_servo_pulsewidth(14, servo_pwm)