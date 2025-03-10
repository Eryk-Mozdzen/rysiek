import json
import serial


class PicoCom:
    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        self.port = port
        self.baud = baudrate
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=1)

    def __del__(self):
        self.ser.close()

    def send_cmd(self, rmot=0, lmot=0, r=0, g=0, b=0):
        json_data = {"rmot": rmot, "lmot": lmot, "r": r, "g": g, "b": b}
        json_string = json.dumps(json_data) + "\n"
        json_encoded = json_string.encode()
        try:
            # print(f"SENDING: {json_string}")
            self.ser.write(json_encoded)
        except serial.SerialException as e:
            print(f"Comm error {e}")
