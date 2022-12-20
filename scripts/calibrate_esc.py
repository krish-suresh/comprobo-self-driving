import pigpio
import time

conn = pigpio.pi()

# print("Calibrating...")
# conn.set_servo_pulsewidth(15, 2000)
# input("Connect power and press Enter to calibrate...")
# conn.set_servo_pulsewidth(15, 2000)
# time.sleep(2)
# conn.set_servo_pulsewidth(15, 1000)
# time.sleep(4)
conn.set_servo_pulsewidth(15, 1500)
time.sleep(4)
