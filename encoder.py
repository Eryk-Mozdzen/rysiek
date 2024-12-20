
def encoder(motor_left: int, motor_right: int):
    M = 0x00
    L = motor_left.to_bytes(1, byteorder='little', signed=True)
    R = motor_right.to_bytes(1, byteorder='little', signed=True)

    checksum = (0x1E + 0xE1 + M + int(L[0]) + int(R[0]) + 0x69) % 256

    bytes = [
        0x1E,
        0xE1,
        checksum,
        M,
        L[0],
        R[0],
        0x69,
    ]

    return bytes

print(encoder(0, 0))
