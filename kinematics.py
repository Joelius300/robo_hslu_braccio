from __future__ import annotations

import math
from typing import Literal, List

import numpy as np
from numpy import ndarray


def rot(axis: Literal['x', 'y', 'z'], angle: int | float):
    """Create a rotation matrix for angle around axis."""

    t = angle * np.pi / 180
    c = cos(t)
    s = sin(t)

    if axis == 'x':
        return np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'y':
        return np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'z':
        return np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    else:
        raise ValueError(f"Axis can only be x, y or z! not {axis}")


def trans(x=0, y=0, z=0):
    """Create a translation matrix for x, y, z."""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


def get_coords_from_matrix(matrix):
    """Returns a vector with the x, y and z coordinates from a 4x4 matrix of homogenous coordinates."""
    return matrix[:3, 3]


def fabrik(points: List[ndarray], segment_lengths: List[float], target: ndarray, max_iterations=100,
           acceptable_distance=.01):
    """
    Finds the points in space where the joints have to be in order for the end effector (last joint) to reach the given
    target (with a given tolerance). Since we already know the lengths of the segments, we pass them to the function to
    avoid calculating them again.
    :param points: List of coordinate-vectors of the joints
    :param segment_lengths: Lengths of the segments between the joints
    :param target: The target coordinate-vector
    :param max_iterations: The maximum number of iterations before a non-converged result is returned
    :param acceptable_distance: The maximum distance the end effector is allowed to be from the target point when returning
    :return: None, because the points array is updated in place
    """
    # FABRIK PROs
    # - fast
    # - simple
    # - smooth motion
    # - realistic motion
    # FABRIK CONs
    # - doesn't return angles but positions
    # - constraints are hard (currently not possible)
    # 1:1 implementation of https://youtu.be/PGk0rnyTa1U?t=221, not pythonized or optimized yet
    origin = points[0]

    for iteration in range(max_iterations):
        starting_from_target = iteration % 2 == 0
        points.reverse()
        segment_lengths.reverse()
        points[0] = target if starting_from_target else origin

        for i in range(1, len(points)):
            direction = norm(points[i] - points[i - 1])
            points[i] = points[i - 1] + direction * segment_lengths[i - 1]

        distance_to_target = vlen(points[-1] - target)
        if not starting_from_target and distance_to_target <= acceptable_distance:
            return


def angle_between(p, q):
    return np.arccos(np.dot(norm(p), norm(q)))


def angle_between_in_plane(p, q, plane_normal):
    """Returns the angle between two vectors in a plane and corrects the handiness according to the plane (aligns
    the directions."""
    cross = np.cross(p, q)
    return angle_between(p, q) * np.sign(np.dot(plane_normal, cross))


def arr(*args):
    return np.array(args)


def vec(*args):
    return np.array([[k] for k in args])


# length of a vector
def vlen(x):
    return np.linalg.norm(x)


# normalize vector to length 1
def norm(x):
    return x / vlen(x)


def cat(*args):
    """Concat vectors side-to-side"""
    return np.hstack(args)


def m(shape, *args):
    """
    From list of numbers, create matrix in the shape of first parameter.

    m((3,4), 1,2,3,4, 5,6,7,8, 9,10,11,12)
    """
    return arr(args).reshape(shape)


def msq(*args):
    """
    From a list of numbers, create a square matrix.

    msq(1,2,3, 4,5,6, 7,8,9)
    """
    sq = math.isqrt(len(args))
    if sq ** 2 != len(args):
        raise ValueError(f'cannot build square matrix with {len(args)} arguments')
    return m((sq, sq), *args)


def sin(x):
    return np.sin(x)


def cos(x):
    return np.cos(x)


def tan(x):
    return np.tan(x)
