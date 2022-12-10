from __future__ import annotations

from io import RawIOBase
from typing import Optional

import numpy as np
import serial
from numpy import ndarray
from serial import Serial

import kinematics
from kinematics import rot, trans, get_coords_from_matrix, angle_between2d, arr, norm, vlen


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

    # lengths from the previous TO THIS JOINT
    # e.g. base to shoulder = 71.5mm, wrist rotation joint to gripper end = 60mm)
    # note that these are all translations in z when the angles are all 0
    JOINT_LENGTHS = {
        'shoulder': 71.5,
        'elbow': 125,
        'wrist_tilt': 125,
        'wrist_rotate': 60,
        'grip': 132,
    }

    # the current CLOCK WISE angles
    current_angles = {
        'base': 0,
        'shoulder': 0,
        'elbow': 0,
        'wrist_tilt': 0,
        'wrist_rotate': 0
    }

    # current x,y,z coordinates of all the joints
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
        # these translations here basically move around the camera
        # we start out at 0,0,0 (looking in x-direction) and then move 280 in x direction.
        # now we're at 280,0,0 so far so good.
        # now we rotate the camera a certain angle COUNTER CLOCK-WISE. When we consider "straight ahead" from our camera
        # to be x-positive we have essentially rotated the whole coordinate system counter clock-wise. This means that
        # all the points which remained on their positions, are obviously now rotated CLOCK-WISE (their position is
        # relative to the x-axis). Here we only want to move around the camera and take snapshots of our positions.
        # but if we wanted to rotate a point around the origin, a counter clock wise rotation of the coordinate system
        # means a clock wise rotation of the point!
        # Another maybe easier way to understand what this does:
        # Imagine you're looking through a camera at position 0,0,0 in x-direction. Then we move around and rotate
        # the coordinate system. If we move the coordinate system 280 in x direction, we are now at -280. But if we say
        # that we as the camera want to stay at 0,0,0 then the origin of the system has now moved the other way and is
        # at 280,0,0. The same goes for rotation. If we keep looking straight and rotate the system around us by x°
        # counter clock wise, we see the system moving to the left (counter clock wise) BUT THAT LOOKS THE SAME AS IF WE
        # ARE ROTATING CLOCK WISE.
        # It's all about perspective and I never want to think about this again holy fuck my brain is fried.
        m1 = trans(x=self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE)  # orig to base
        m2 = rot('z', self.current_angles['base']) @ trans(z=self.JOINT_LENGTHS['shoulder'])  # base to shoulder
        m3 = rot('y', self.current_angles['shoulder']) @ trans(z=self.JOINT_LENGTHS['elbow'])  # shoulder to elbow
        m4 = rot('y', self.current_angles['elbow']) @ trans(z=self.JOINT_LENGTHS['wrist_tilt'])  # elbow to wrist_tilt
        m5 = rot('y', self.current_angles['wrist_tilt']) @ trans(z=self.JOINT_LENGTHS['wrist_rotate'])  # wrist_tilt to wrist_rotate
        m6 = rot('z', self.current_angles['wrist_rotate']) @ trans(z=self.JOINT_LENGTHS['grip'])  # wrist_rotate to grip

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
        angles['shoulder'] = -np.rad2deg(angle_between2d(arr(0, 1), shoulder_to_elbow))  # left handed -> negative

        elbow_to_wrist = points['wrist_tilt'] - points['elbow']
        angles['elbow'] = -np.rad2deg(angle_between2d(shoulder_to_elbow, elbow_to_wrist))  # left handed -> negative

        wrist_to_end = points['grip'] - points['wrist_tilt']  # ignore wrist_rotate, must be on the same line anyway
        angles['wrist_tilt'] = -np.rad2deg(angle_between2d(elbow_to_wrist, wrist_to_end))  # left handed -> negative

        return angles

    def get_calculated_angles(self):
        points = self._get_rotated_points_on_x_z()

        angles = self._get_angles_from_points2d(points)
        angles.update(base=self.current_angles['base'])

        return angles

    def _get_rotated_points_on_x_z(self):
        [x, _, z] = self.current_points['base']  # no need to rotate base around itself, pointless
        points = {
            'base': arr(x, z),
            'shoulder': self._get_point_on_x_z('shoulder'),
            'elbow': self._get_point_on_x_z('elbow'),
            'wrist_tilt': self._get_point_on_x_z('wrist_tilt'),
            'grip': self._get_point_on_x_z('grip')
        }

        print("2d points")
        print(points)

        return points

    def _rotate_around_base_to_x_z(self, p, angle):
        """Rotates point around the base CLOCK WISE for the purpose of bringing the point into the x-z plane."""
        moved_to_origin = trans(-self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE, 0, 0) @ trans(*p)
        rotated = rot('z', angle) @ moved_to_origin
        moved_back = trans(self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE, 0, 0) @ rotated
        moved_back = get_coords_from_matrix(moved_back)

        # we only rotate around z so the x should now be the previous hypotenuse in the x-y
        # and y should be 0. z should not have moved.
        [x, y, z] = (moved_back - self.current_points['base'])
        [xp, yp, zp] = (p - self.current_points['base'])
        hyp = np.sqrt(xp ** 2 + yp ** 2) * np.sign(xp)
        if not np.isclose(hyp, x):
            print(f"After rotation, the point was expected to have the x of the prev. hypotenuse {hyp} but has {x} instead.")

        if not np.isclose(y, 0):
            print(f"After rotation, the point was expected to be on the x-z plane (y=0) but has y={y}")

        if not np.isclose(z, zp):
            print(f"After rotation, the point was expected to have kept its z position but it changed from {zp} to {z}")

        return moved_back

    def _get_point_on_x_z(self, name: str):
        # the base angle is clock wise and the rotate_around_base method rotates the point clock wise.
        # since we want to move the point "back" and compensate the base-rotation, we negate it so it
        # a counter clock wise rotation which nullifies the current clock wise rotation
        angle_needed_to_move_arm_to_x_z_plane = -self.current_angles['base']
        rotated = self._rotate_around_base_to_x_z(self.current_points[name], angle_needed_to_move_arm_to_x_z_plane)
        [x, y, z] = rotated
        if not np.isclose(0, y):
            print(f"After rotating {name} by {-self.current_angles['base']}° to the x-z plane we expected y to be 0 but it was {y}!")

        return arr(x, z)

    def fabrik(self, target: ndarray, acceptable_distance=.1):
        """
        Uses the FABRIK algorithm to determine an angle configuration that would move the end effector
        of Braccio to the specified target smoothly while taking into account the current position.

        Note on implementation:

        Since all of Braccio's joins except for the base (and wrist_rotate, but we don't care about that since it doesn't
        influence the position of the arm) operate only in one plane, we actually do a 2D FABRIK.
        For this we first rotate all the current joint positions to the x-z plane by doing a rotation in the opposite
        direction that the base is currently rotated in. This means all the points y positions will be 0 and can be
        dropped but unlike if we had projected, the distances between the points remain the same.
        We also do this for the target, so we have a point on the x-z plane that we can target, which has still the same
        distance from the base as the real target.
        Then after FABRIK is done, we use the 2D positions output (of FABRIK) to calculate the angles of the joins
        in this x-z plane (could be any plane, the vectors are now 2d and don't know about any external plane in 3d).
        These angles are obviously the same as in 3d because we are basically just looking at the arm from an angle
        that is perpendicular to the arm's plane, or another way of interpreting it (which is even closer to what we're
        actually doing) is that we stand beside the arm, rotate it so the base is in it's default position with the arm
        going straight along the x-axis, squint one eye so we only see the arm in 2d, and calculate the angles between
        the arm parts. These angles don't change if we turn the base so these will be the ones we need to use even
        though we calculated them in 2d and use it in 3d. All because the arm only operates in one plane. makes sense?
        The wrist_rotate angle cannot be determined because it doesn't have an effect on the position of the gripper.
        The base angle was already determined at the beginning using a transformed dot product formula.

        :param target: The coordinates of the target point as a 3d row vector.
        :param acceptable_distance: The maximum tolerated distance from the potential end effector position to the target position.
        :return: A configuration of angles for the Braccio. Can be sent to Braccio with send(**angles).
        """

        base_to_target = target - self.current_points['base']
        base_to_target[2] = 0  # project to x-y plane
        base_to_target = norm(base_to_target)
        # see formula in chat. it's basically just the dot product transformed.
        required_base_angle = -np.rad2deg((np.sign(base_to_target[1])) * np.arccos(
            -base_to_target[0] / np.sqrt((base_to_target[0] ** 2) + (base_to_target[1] ** 2))))

        l = lambda name: self.JOINT_LENGTHS[name]
        points = list(self._get_rotated_points_on_x_z().values())
        lengths = [l('shoulder'), l('elbow'), l('wrist_tilt'), l('wrist_rotate') + l('grip')]

        # sanity check
        # we are rotating, not projecting, so the distances should be the same (they also need to be for FABRIK)!
        for i in range(len(points) - 1):
            v = points[i + 1] - points[i]
            l = vlen(v)
            if not np.isclose(l, lengths[i]):
                print(f"Expected length of {lengths[i]} but got {l}!!")

        distance_base_to_target_before = vlen(target - self.current_points['base'])

        # also rotate target onto the x-z plane
        # instead of moving the arm to the target (arm clock wise rotation around base) we want to move the target
        # to the x-z plane (target counter clock wise rotation around base) therefore we negate the angle.
        target = self._rotate_around_base_to_x_z(target, -required_base_angle)

        distance_base_to_target_after = vlen(target - self.current_points['base'])

        if not np.isclose(distance_base_to_target_before, distance_base_to_target_after):
            print(f"After rotating target by {-required_base_angle} we expected the distance to still be {distance_base_to_target_before} but it changed to {distance_base_to_target_after}!")

        [x, _, z] = target

        target = arr(x, z)
        print("Rotated target")
        print(target)

        kinematics.fabrik(points, lengths, target, acceptable_distance=acceptable_distance)
        points = {
            'base': points[0],
            'shoulder': points[1],
            'elbow': points[2],
            'wrist_tilt': points[3],
            'grip': points[4],
        }

        print("FABRIK points")
        print(points)

        self._ensure_fabrik_makes_sense(points)

        angles = self._get_angles_from_points2d(points)
        angles.update(base=required_base_angle)

        return angles

    def _ensure_fabrik_makes_sense(self, points):
        # this code is fucking disgusting but who cares at this point
        # TODO also assert positions, not just lengths, or is that redundant?
        if not np.allclose(points['base'], arr(self.DISTANCE_FROM_OUR_ORIGIN_TO_BASE, 0)):
            print(f"Base not where we expect but {points['base']}")

        base_to_shoulder = points['shoulder'] - points['base']
        base_to_shoulder_len = vlen(base_to_shoulder)
        if not np.isclose(base_to_shoulder_len, self.JOINT_LENGTHS['shoulder']):
            print(f"Base to shoulder length unexpected: {base_to_shoulder_len} instead of {self.JOINT_LENGTHS['shoulder']}")

        shoulder_to_elbow = points['elbow'] - points['shoulder']
        shoulder_to_elbow_len = vlen(shoulder_to_elbow)
        if not np.isclose(shoulder_to_elbow_len, self.JOINT_LENGTHS['elbow']):
            print(f"Shoulder to elbow length unexpected: {shoulder_to_elbow_len} instead of {self.JOINT_LENGTHS['elbow']}")

        elbow_to_wrist_tilt = points['wrist_tilt'] - points['elbow']
        elbow_to_wrist_tilt_len = vlen(elbow_to_wrist_tilt)
        if not np.isclose(elbow_to_wrist_tilt_len, self.JOINT_LENGTHS['wrist_tilt']):
            print(f"Elbow to wrist tilt length unexpected: {elbow_to_wrist_tilt_len} instead of {self.JOINT_LENGTHS['wrist_tilt']}")

        wrist_tilt_to_grip = points['grip'] - points['wrist_tilt']
        wrist_tilt_to_grip_len = vlen(wrist_tilt_to_grip)
        if not np.isclose(wrist_tilt_to_grip_len, self.JOINT_LENGTHS['wrist_rotate'] + self.JOINT_LENGTHS['grip']):
            print(f"Wrist tilt to grip length unexpected: {wrist_tilt_to_grip_len} instead of {self.JOINT_LENGTHS['wrist_rotate'] + self.JOINT_LENGTHS['grip']}")

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
