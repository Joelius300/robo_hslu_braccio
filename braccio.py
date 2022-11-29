from __future__ import annotations

from io import RawIOBase
from typing import Optional

import serial
from serial import Serial

from kinematics import rot, trans, get_coords_from_matrix


def _transform_mm_to_grip(mm: Optional[int]):
    if mm is None:
        return None

    return max(73 - mm, 10)


class Braccio:
    """
    Class to communicate with an Arduino controlled Braccio using the Postfix protocol by J.W. (see Braccio folder).
    """
    KEY_MAPPINGS = {
        'base': 'b',
        'shoulder': 's',
        'elbow': 'e',
        'wrist_tilt': 'v',
        'wrist_rotate': 'w',
        'grip': 'g'
    }

    ANGLE_CORRECTIONS = {
        'base': 90,
        'shoulder': 90,
        'elbow': 90,
        'wrist_tilt': 95,
        'wrist_rotate': 90
    }

    # lengths from one joint to the next (always the previous one in the list
    # e.g. base to shoulder = 71.5mm, wrist rotation joint to gripper end = 60mm)
    # note that these are all translations in z
    JOINT_LENGTHS = {
        'shoulder': 71.5,
        'elbow': 125,
        'wrist_tilt': 125,
        'wrist_rotate': 60,
        'grip': 132,
    }

    DISTANCE_FROM_OUR_ORIGIN_TO_BASE = 280  # mm

    current_angles = {
        'base': 0,
        'shoulder': 0,
        'elbow': 0,
        'wrist_tilt': 0,
        'wrist_rotate': 0
    }

    def __init__(self, port: str | Serial):
        if isinstance(port, str):
            self._port = serial.Serial(port, baudrate=9600, parity='N', bytesize=8, stopbits=1, timeout=None)
        else:
            self._port = port

    def send(self, *,
             base: Optional[int] = None,
             shoulder: Optional[int] = None,
             elbow: Optional[int] = None,
             wrist_tilt: Optional[int] = None,
             wrist_rotate: Optional[int] = None,
             grip: Optional[int] = None) -> int:
        """
        Sets the specified angles of the Braccio joints over the Arduino.

        Note, the grip is passed as mm distance of the inner claw. To grip something of a certain width, you'll
        need to clamp 10-15mm tighter because they are flexible and don't hold the object if it matches the width.
        """

        grip = _transform_mm_to_grip(grip)

        self.current_angles = {
            'base': base,
            'shoulder': shoulder,
            'elbow': elbow,
            'wrist_tilt': wrist_tilt,
            'wrist_rotate': wrist_rotate
          }

        return self._send(base=base, shoulder=shoulder,
                          elbow=elbow, wrist_tilt=wrist_tilt,
                          wrist_rotate=wrist_rotate, grip=grip)

    def reset_position(self):
        self.send(base=0, shoulder=0, elbow=0, wrist_tilt=0, wrist_rotate=0, grip=40)  # default position

    def get_end_point(self):

        m1 = trans(x=self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE) @ rot('z', self.current_angles['base'])
        m2 = trans(z=self.JOINT_LENGTHS['shoulder']) @ rot('y', self.current_angles['shoulder'])
        m3 = trans(z=self.JOINT_LENGTHS['elbow']) @ rot('y', self.current_angles['elbow'])
        m4 = trans(z=self.JOINT_LENGTHS['wrist_tilt']) @ rot('y',  self.current_angles['wrist_tilt'])
        m5 = trans(z=self.JOINT_LENGTHS['wrist_rotate']) @ rot('z', self.current_angles['wrist_rotate'])
        m6 = trans(z=self.JOINT_LENGTHS['grip'])

        M = m1 @ m2 @ m3 @ m4 @ m5 @ m6

        return get_coords_from_matrix(M)


    def _send(self, **kwargs) -> int:
        payload = self._build_payload(**kwargs)
        print("Sending:", payload)
        payload += '\r\n'
        b = payload.encode('ASCII')
        print("Bytes:", b.hex())

        return self._port.write(b)

    def _build_payload(self, **kwargs):
        payload = ''
        for key, value in kwargs.items():
            if value is not None:
                correction = self.ANGLE_CORRECTIONS.get(key)
                if correction:
                    value += correction

                payload += f'{value} '
                payload += f'{self.KEY_MAPPINGS[key]} '

        return payload.rstrip()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self._port.close()


if __name__ == '__main__':

    port = serial.serial_for_url("loop://", timeout=.1)  # testing
    # port = "COM4"  # windows
    # port = "/dev/ttyACM0"  # linux
    with Braccio(port) as braccio:
        braccio.send(base=0, shoulder=30, elbow=20, wrist_tilt=10, wrist_rotate=0)

        print(braccio.get_end_point())
        if isinstance(port, RawIOBase):
            print("Dumping received data on port:")
            print(port.readall().decode('ASCII'))
