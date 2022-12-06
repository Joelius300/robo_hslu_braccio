from __future__ import annotations

from io import RawIOBase
from typing import Optional

import numpy as np
import serial
from numpy import ndarray
from serial import Serial

import kinematics
from kinematics import rot, trans, get_coords_from_matrix, angle_between_in_plane, arr, norm


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

    def get_calculated_angles(self):
        """
        Returns the angles that are calculated from the current estimated position.
        Note this is wildly imprecise as it's a double estimation first from angles to position then back to angles.
        However, if our code is sound, the difference between these angles and the ones passed into send should be
        very small.
        """
        return self._get_angles_from_points(self.current_points)

    def _get_angles_from_points(self, points: dict[str, ndarray]):
        angles = {}
        base_to_end = points['grip'] - points['base']
        base_to_end[2] = 0  # project to x-y plane
        base_to_end = norm(base_to_end)
        # angles['base'] = np.rad2deg(np.arctan2(base_to_end[1], base_to_end[0])) AFAIK this does the same..
        # see formula in chat. it's basically just the dot product transformed.
        angles['base'] = -np.rad2deg((np.sign(base_to_end[1])) * np.arccos(
            -base_to_end[0] / np.sqrt((base_to_end[0] ** 2) + (base_to_end[1] ** 2))))

        # the angle of the base in the left hand system is clock-wise respective to the positive x-axis
        # therefore the normal of the plane that the arm is on when the base is rotated by that angle,
        # is the vector 0,1,0 rotated by this angle clock-wise. That gives us sin(-p), cos(-p), 0.
        # The reason it's -p, is because normally you'd rotate counter clock-wise, so we have to invert the direction.
        arm_plane_normal = arr(np.sin(-np.deg2rad(angles['base'])), np.cos(-np.deg2rad(angles['base'])), 0)

        # for all the others we take the angle between two vectors since they might not be in a plane when the base
        # is rotated (otherwise we could theoretically use arctan).
        # When we take the angle between two vectors using the dot product of them we don't need to take
        # this potential base rotation into account. Uses the very important formula: v * w = |v| * |w| * cos(p)  :)
        shoulder_to_elbow = points['elbow'] - points['shoulder']
        angles['shoulder'] = np.rad2deg(angle_between_in_plane(arr(0, 0, 1), shoulder_to_elbow, arm_plane_normal))
        elbow_to_wrist = points['wrist_tilt'] - points['elbow']
        angles['elbow'] = np.rad2deg(angle_between_in_plane(shoulder_to_elbow, elbow_to_wrist, arm_plane_normal))
        # could also use wrist_rotate instead of grip, will be on the same line, wrist_rotate can only rotate in z
        wrist_to_end = points['grip'] - points['wrist_tilt']
        angles['wrist_tilt'] = np.rad2deg(angle_between_in_plane(elbow_to_wrist, wrist_to_end, arm_plane_normal))
        # wrist_rotate's angle doesn't influence end position, so it cannot be determined here

        # Turn the arm around if the angles want to turn the base more than 90°
        # IMO this >= doesn't really make sense it should only be > 90 because that's the restriction on the robot.
        # However, when arccos could return 90 or -90 I assume it always returns 90 so when you input -90 and it
        # recalculates, it comes out to 90. With >= this doesn't happen but I'm not sure if there might be other
        # edge cases produced by this. Anyway it seems to work fine now, both 90 and -90 work too.
        if abs(angles['base']) >= 90:
            angles['base'] += -np.sign(angles['base']) * 180  # add 180 if negative, subtract 180 if positive
            angles['shoulder'] *= -1
            angles['elbow'] *= -1
            angles['wrist_tilt'] *= -1

        return angles

    def fabrik(self, target: ndarray, acceptable_distance=.5):
        """
        Uses the FABRIK algorithm to determine an angle configuration that would move the end effector
        of Braccio to the specified target smoothly while taking into account the current position.

        :param target: The coordinates of the target point as a 3d row vector.
        :param acceptable_distance: The maximum tolerated distance from the potential end effector position to the target position.
        :return: A configuration of angles for the Braccio. Can be sent to Braccio with send(**angles).
        """
        p = lambda x: self.current_points[x]
        l = lambda x: self.JOINT_LENGTHS[x]
        points = [p('base'), p('shoulder'), p('elbow'), p('wrist_tilt'), p('grip')]
        lengths = [l('shoulder'), l('elbow'), l('wrist_tilt'), l('wrist_rotate') + l('grip')]
        kinematics.fabrik(points, lengths, target, acceptable_distance=acceptable_distance)
        points = {
            'base': points[0],
            'shoulder': points[1],
            'elbow': points[2],
            'wrist_tilt': points[3],
            'grip': points[4],
        }

        return self._get_angles_from_points(points)

    def _send(self, **kwargs) -> int:
        self.current_angles.update({k: v for k, v in kwargs.items() if v is not None})
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
