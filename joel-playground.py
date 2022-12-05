import time
from io import RawIOBase

import serial

from braccio import Braccio
from kinematics import arr

if __name__ == '__main__':
    port = "/dev/ttyACM0"
    # port = serial.serial_for_url("loop://", timeout=.1)  # testing
    with Braccio(port) as b:
        time.sleep(5)  # important

        # b.send(base=25, shoulder=30, elbow=15, wrist_tilt=10, wrist_rotate=20)
        angles = b.fabrik(arr(50, 50, 200))
        # it somehow goes too low with this..
        b.send(**angles)
        print(b.current_angles)

        if isinstance(port, RawIOBase):
            print("Dumping received data on port:")
            print(port.readall().decode('ASCII'))

        time.sleep(10)
        b.reset_position()


def hard_coded_pick_and_place(b: Braccio):
    """DON'T RUN THIS, this was before translations."""
    # 120 b 75 s 20 e 20 v 90 w 0 g
    # 55 g
    # 90 s
    # 75 b
    # 75 s
    # 0 g
    return None

    b.send(base=120, shoulder=75, elbow=20, wrist_tilt=20, wrist_rotate=90, grip=100)
    time.sleep(5)
    b.send(grip=20)
    time.sleep(2)
    b.send(shoulder=90)
    time.sleep(2)
    b.send(base=75)
    time.sleep(2)
    b.send(shoulder=75)
    time.sleep(2)
    b.send(grip=100)
