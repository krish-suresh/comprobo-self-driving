import serial


class ArduinoReceiver():
    def __init__(self):
        self.initialize_serial()

    def initialize_serial(self):
        self.ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

test = ArduinoReceiver()
while True:
    input = test.ser.readline()
    try:
        t = input.decode().strip()
        print(int(t))
    except (UnicodeDecodeError, ValueError):
        continue