from __future__ import annotations

from io import RawIOBase
from typing import Optional

import numpy as np
import serial
from numpy import ndarray
from serial import Serial

import kinematics
from kinematics import rot, trans, get_coords_from_matrix, angle_between, arr, norm, vlen


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
        self._calculate_points_from_angles()

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

    # forward kinematics
    def _calculate_points_from_angles(self):
        m1 = trans(x=self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE) @ rot('z', self.current_angles['base'])
        m2 = trans(z=self.JOINT_LENGTHS['shoulder']) @ rot('y', self.current_angles['shoulder'])
        m3 = trans(z=self.JOINT_LENGTHS['elbow']) @ rot('y', self.current_angles['elbow'])
        m4 = trans(z=self.JOINT_LENGTHS['wrist_tilt']) @ rot('y', self.current_angles['wrist_tilt'])
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

    def _get_angles_from_points2d(self, points: dict[str, ndarray]):
        angles = {}

        shoulder_to_elbow = points['elbow'] - points['shoulder']
        angles['shoulder'] = -np.rad2deg(angle_between(arr(0, 1), shoulder_to_elbow))  # left handed -> negative

        elbow_to_wrist = points['wrist_tilt'] - points['elbow']
        angles['elbow'] = -np.rad2deg(angle_between(shoulder_to_elbow, elbow_to_wrist))  # left handed -> negative

        wrist_to_end = points['grip'] - points['wrist_tilt']  # ignore wrist_rotate, must be on the same line anyway
        angles['wrist_tilt'] = -np.rad2deg(angle_between(elbow_to_wrist, wrist_to_end))  # left handed -> negative

        return angles

    def fabrik(self, target: ndarray, acceptable_distance=.1):
        """
        Uses the FABRIK algorithm to determine an angle configuration that would move the end effector
        of Braccio to the specified target smoothly while taking into account the current position.

        Note on implementation:

        Since all of Braccio's joins except for the base operate only in one plane, we actually do a 2D FABRIK
        by projecting all the current joint positions as well as the target onto the x-z plane before doing FABRIK.
        Then after FABRIK is done, we use these 2D positions to calculate the angles of the joins in this plane.
        The wrist_rotate angle cannot be determined because it doesn't have an effect on the position of the gripper.
        The base angle was already determined at the beginning using a transformed dot product formula.

        :param target: The coordinates of the target point as a 3d row vector.
        :param acceptable_distance: The maximum tolerated distance from the potential end effector position to the target position.
        :return: A configuration of angles for the Braccio. Can be sent to Braccio with send(**angles).
        """

        base_to_end = target - self.current_points['base']
        base_to_end[2] = 0  # project to x-y plane
        base_to_end = norm(base_to_end)
        # see formula in chat. it's basically just the dot product transformed.
        required_base_angle = -np.rad2deg((np.sign(base_to_end[1])) * np.arccos(
            -base_to_end[0] / np.sqrt((base_to_end[0] ** 2) + (base_to_end[1] ** 2))))

        rotate_to_x_z_matrix = rot('z', -required_base_angle)

        def p(name):
            [x, y, z] = self.current_points[name]
            rotated = rotate_to_x_z_matrix @ trans(x, y, z)
            [x, y, z] = get_coords_from_matrix(rotated)
            if not np.isclose(0, y):
                print(f"After rotating {name} by {-required_base_angle} we expected y to be 0 but it was {y}!")

            return arr(x, z)

        l = lambda name: self.JOINT_LENGTHS[name]
        points = [p('base'), p('shoulder'), p('elbow'), p('wrist_tilt'), p('grip')]
        lengths = [l('shoulder'), l('elbow'), l('wrist_tilt'), l('wrist_rotate') + l('grip')]

        # sanity check
        # we are rotating, not projecting so the distances should be the same (they also need to be for FABRIK)!
        for i in range(len(points) - 1):
            v = points[i + 1] - points[i]
            l = vlen(v)
            if not np.isclose(l, lengths[i]):
                print(f"Expected length of {lengths[i]} but got {l}!!")

        [x, y, z] = target
        [x, y, z] = get_coords_from_matrix(rotate_to_x_z_matrix @ trans(x, y, z))

        if not np.isclose(0, y):
            print(f"After rotating target by {-required_base_angle} we expected y to be 0 but it was {y}!")

        target = arr(x, z)
        print("Rotated target")
        print(target)

        kinematics.fabrik(points, lengths, target, acceptable_distance=acceptable_distance)
        points = {
            'shoulder': points[1],
            'elbow': points[2],
            'wrist_tilt': points[3],
            'grip': points[4],
        }
        print("FABRIK points")
        print(points)

        angles = self._get_angles_from_points2d(points)
        angles.update(base=required_base_angle)

        return angles

    def _send(self, **kwargs) -> int:
        self.current_angles.update({k: v for k, v in kwargs.items() if v is not None})
        self._calculate_points_from_angles()

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

                payload += f'{int(value)} '
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
