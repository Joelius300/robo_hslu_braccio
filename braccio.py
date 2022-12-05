from __future__ import annotations

import math
from io import RawIOBase
from typing import Optional, List

import numpy as np
import serial
from numpy import ndarray
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
        'wrist_tilt': 90 + 5,
        'wrist_rotate': 90
    }

    DISTANCE_FROM_OUR_ORIGIN_TO_BASE = 280  # mm

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

    current_angles = {
        'base': 0,
        'shoulder': 0,
        'elbow': 0,
        'wrist_tilt': 0,
        'wrist_rotate': 0
    }

    current_points: dict[str, ndarray] = {}

    def __init__(self, port: str | Serial):
        if isinstance(port, str):
            self._port = serial.Serial(port, baudrate=9600, parity='N', bytesize=8, stopbits=1, timeout=None)
        else:
            self._port = port
        self._calculate_points()

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

        return self._send(base=base, shoulder=shoulder,
                          elbow=elbow, wrist_tilt=wrist_tilt,
                          wrist_rotate=wrist_rotate, grip=grip)

    def reset_position(self):
        self.send(base=0, shoulder=0, elbow=0, wrist_tilt=0, wrist_rotate=0, grip=40)  # default position

    def _calculate_points(self):
        m1 = trans(x=self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE) @ rot('z', self.current_angles['base'])
        m2 = trans(z=self.JOINT_LENGTHS['shoulder']) @ rot('y', self.current_angles['shoulder'])
        m3 = trans(z=self.JOINT_LENGTHS['elbow']) @ rot('y', self.current_angles['elbow'])
        m4 = trans(z=self.JOINT_LENGTHS['wrist_tilt']) @ rot('y',  self.current_angles['wrist_tilt'])
        m5 = trans(z=self.JOINT_LENGTHS['wrist_rotate']) @ rot('z', self.current_angles['wrist_rotate'])
        m6 = trans(z=self.JOINT_LENGTHS['grip'])

        M = m1
        self.current_points['base'] = get_coords_from_matrix(M)
        M = M @ m2
        self.current_points['shoulder'] = get_coords_from_matrix(M)
        M = M @ m3
        self.current_points['elbow'] = get_coords_from_matrix(M)
        M = M @ m4
        self.current_points['wrist_tilt'] = get_coords_from_matrix(M)
        M = M @ m5
        self.current_points['wrist_rotate'] = get_coords_from_matrix(M)
        M = M @ m6
        self.current_points['grip'] = get_coords_from_matrix(M)

    def get_end_point(self):
        return self.current_points['grip']

    def _get_angles_from_points(self, points: dict[str, ndarray]):
        angles = {}
        base_to_end = points['grip'] - points['base']
        angles['base'] = math.degrees(np.arctan2(base_to_end[1], base_to_end[0]))
        shoulder_to_elbow = points['elbow'] - points['shoulder']
        angles['shoulder'] = math.degrees(np.arctan2(shoulder_to_elbow[0], shoulder_to_elbow[2]))
        elbow_to_wrist = points['wrist_tilt'] - points['elbow']
        angles['elbow'] = math.degrees(np.arctan2(elbow_to_wrist[0], elbow_to_wrist[2])) - angles['shoulder']
        wrist_to_end = points['grip'] - points['wrist_tilt']  # could also use wrist_rotate instead of grip, will be on the same line because wrist_rotate can only rotate in z
        angles['wrist_tilt'] = math.degrees(np.arctan2(wrist_to_end[0], wrist_to_end[2])) - angles['elbow']

        # wrist_rotate's angle doesn't influence end position, so it cannot be determined here
        return angles

    def get_angles_from_points(self):
        return self._get_angles_from_points(self.current_points)

    def _send(self, **kwargs) -> int:
        self.current_angles.update(kwargs)
        self._calculate_points()

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
