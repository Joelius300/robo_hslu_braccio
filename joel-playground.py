import random
import time
from io import RawIOBase

import serial

from braccio import Braccio
from kinematics import arr
import random


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
        for k, v in calc_angles.items():
            expected = round(angles[k])
            actual = round(v)
            if actual != expected:
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


if __name__ == '__main__':
    port = "/dev/ttyACM0"
    port = serial.serial_for_url("loop://", timeout=.1)  # testing
    with Braccio(port) as b:
        # time.sleep(5)  # important

        # testCase = {'base': 90, 'shoulder': 82, 'elbow': 66, 'wrist_tilt': -84}
        # b.send(**testCase)
        b.send(base=-72, shoulder=-30, elbow=15, wrist_tilt=10)
        # print("Input angles")
        # print(b.current_angles)
        # print("Recalculated angles")
        # print(b.get_calculated_angles())

        print("Testing FABRIK")
        # it somehow goes too low with this..
        # target = arr(100, -200, 44)
        target = arr(50, -50, 200)
        print("Target:", target)
        print("Current position")
        print(b.current_points)
        angles = b.fabrik(target)
        print("FABRIK angles")
        print(angles)
        b.send(**angles)
        print("Input angles")
        print(b.current_angles)
        # print("Recalculated angles")
        # print(b.get_calculated_angles())
        print(f"Current points (grip should be at {target})")
        print(b.current_points)

        # Okay we vorhär d Position reset isch de isch stimmt z target mit FABRIK

        if isinstance(port, RawIOBase):
            print("Dumping received data on port:")
            # print(port.readall().decode('ASCII'))
        else:
            time.sleep(10)
            b.reset_position()
