import serial

MAGIC1 = 0x1E
MAGIC2 = 0xE1
MAGIC3 = 0x69


def encoder(matrix_num: int, motor_left: int, motor_right: int):
    M = matrix_num.to_bytes(1, byteorder="little", signed=True)
    L = motor_left.to_bytes(1, byteorder="little", signed=True)
    R = motor_right.to_bytes(1, byteorder="little", signed=True)

    checksum = (0x1E + 0xE1 + int(M[0]) + int(L[0]) + int(R[0]) + 0x69) % 256

    data = [
        0x1E,
        0xE1,
        checksum,
        M[0],
        L[0],
        R[0],
        0x69,
    ]
    return data


class NucleoCom:
    def __init__(self, port="/dev/ttyTHS1", baudrate=9600):
        self.port = port
        self.baud = baudrate
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )

    def __del__(self):
        self.ser.close()

    def send_cmd(self, matrix_num=0, lmot=0, rmot=0):
        lmot = max(-100, min(100, lmot))
        rmot = max(-100, min(100, rmot))
        data = encoder(int(matrix_num), int(lmot), int(rmot))

        try:
            print(f"SENDING: {data}")
            self.ser.write(data)
        except serial.SerialException as e:
            print(f"Comm error {e}")
