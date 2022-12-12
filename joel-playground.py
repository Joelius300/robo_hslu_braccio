import random
import time
from io import RawIOBase

import numpy as np

from braccio import Braccio
from kinematics import arr


def test_angle_conversions(b: Braccio):
    for i in range(10000):
        # break
        angles = {
            'base': random.randint(-90, 90),
            'shoulder': random.randint(-90, 90),
            'elbow': random.randint(-90, 90),
            'wrist_tilt': random.randint(-90, 90),
        }
        b.send(**angles)
        calc_angles = b.get_calculated_angles()
        input_printed = False
        # print("Input: ", angles)
        # print("Output: ", calc_angles)
        for k, v in calc_angles.items():
            expected = angles[k]
            actual = v
            if not np.isclose(actual, expected):
                if not input_printed:
                    print("Input: ", angles)
                    print("Output: ", calc_angles)
                    input_printed = True
                print(f"ERROR: {k} should have been {expected} but was {actual}")
        time.sleep(0.05)


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


def send_fabrik(b: Braccio, position, grip=None):
    print("Sending position", position)
    angles = b.fabrik(position)
    if grip:
        angles.update(grip=grip)
    print("With angles", angles)
    b.send(**angles)


# corner to corner
def pick_n_place(braccio: Braccio):
    a = arr(5, -165, 30)
    b = arr(-10, 0, 300)
    c = arr(15, 155, 25)

    send_fabrik(braccio, a, grip=60)
    time.sleep(5)
    braccio.send(grip=0)
    time.sleep(1)
    send_fabrik(braccio, a + arr(0, 0, 100))
    time.sleep(1)
    send_fabrik(braccio, b)
    time.sleep(2)
    braccio.send(wrist_rotate=70)
    time.sleep(1.5)
    braccio.send(wrist_rotate=-70)
    time.sleep(1.5)
    braccio.send(wrist_rotate=0)
    time.sleep(1)
    send_fabrik(braccio, c)
    time.sleep(3)
    braccio.send(grip=100)
    time.sleep(2)
    send_fabrik(braccio, c + arr(0, 0, 100))
    time.sleep(2)
    braccio.reset_position()


if __name__ == '__main__':
    port = "/dev/ttyACM0"
    # port = serial.serial_for_url("loop://", timeout=.1)  # testing
    with Braccio(port) as b:
        time.sleep(5)  # important

        try:
            pick_n_place(b)

            input("Press enter to reset..")
        finally:
            if isinstance(port, RawIOBase):
                print("Dumping received data on port:")
                # print(port.readall().decode('ASCII'))
            else:
                b.reset_position()
