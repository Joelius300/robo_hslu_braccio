import random
import time
from io import RawIOBase

import serial
from pynput.keyboard import Key, KeyCode

from braccio import Braccio
from kinematics import arr
import random
import numpy as np
from pynput import keyboard

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


def wasd(b: Braccio):
    position = b.get_end_point()
    max_send_interval = 5
    last_send = [-1]
    should_end = [False]
    move_step = 5
    print("Starting controller from", position)

    def on_press(key: Key | KeyCode) -> bool | None:
        if key == KeyCode.from_char('x'):
            should_end[0] = True
            return False

        if key == KeyCode.from_char('w'):
            position[0] += move_step
        elif key == KeyCode.from_char('s'):
            position[0] -= move_step
        elif key == KeyCode.from_char('d'):
            position[1] += move_step
        elif key == KeyCode.from_char('a'):
            position[1] -= move_step
        elif key == Key.space:
            position[2] += move_step
        elif key == Key.shift:
            position[2] -= move_step
        else:
            return True

        print("Position now:", position)
        return True

        now = time.time()
        if last_send[0] + max_send_interval <= now:
            # print("Sending position", position)
            b.send(**b.fabrik(position))
            last_send[0] = now
        else:
            print(f"Cannot send, only {now - last_send[0]} seconds passed since last send.")

    with keyboard.Listener(on_press=on_press) as listener:
        last_position = position.copy()
        while not should_end[0]:
            time.sleep(max_send_interval)
            pos = position.copy()
            if last_position is None or not np.allclose(pos, last_position):
                print("Sending position", pos)
                angles = b.fabrik(pos)
                print("Sending angles", angles)
                b.send(**angles)
                last_position = pos


if __name__ == '__main__':
    port = "/dev/ttyACM0"
    # port = serial.serial_for_url("loop://", timeout=.1)  # testing
    with Braccio(port) as b:
        time.sleep(5)  # important

        try:
            wasd(b)
        finally:
            b.reset_position()

        # target = arr(40, 100, 50)
        # angles = b.fabrik(target)
        # print("Calculated angles from FABRIK")
        # print(angles)
        # b.send(**angles)

        # if isinstance(port, RawIOBase):
        #     print("Dumping received data on port:")
        #     # print(port.readall().decode('ASCII'))
        # else:
        #     time.sleep(15)
        #     b.reset_position()
