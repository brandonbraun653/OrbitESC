# **********************************************************************************************************************
#   FileName:
#       serial_pipe.py
#
#   Description:
#       Serial pipe to communicate with an OrbitESC device
#
#   12/12/2022 | Brandon Braun | brandonbraun653@gmail.com
# **********************************************************************************************************************

import serial


class SerialPipe:
    """ Message server for RTX-ing Protocol Buffer over a serial connection """

    def __init__(self):
        pass


if __name__ == "__main__":
    ser = serial.Serial(port="/dev/ttyUSB0", baudrate=921600, timeout=5)
    data = ser.read(256)
    print(data)