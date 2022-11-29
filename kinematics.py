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


def fabrik(points: List[ndarray], target: ndarray, max_iterations=100, min_acceptable_distance=.01):
    # 1:1 implementation of https://youtu.be/PGk0rnyTa1U?t=221, not pythonized or optimized yet
    origin = points[0]
    segment_lengths = []
    for i in range(len(points) - 1):
        segment_lengths[i] = vlen(points[i + 1] - points[i])

    for iteration in range(max_iterations):
        starting_from_target = iteration % 2 == 0
        points.reverse()
        segment_lengths.reverse()
        if starting_from_target:  # shouldn't this always be the case
            assert points[0] == target
        else:
            assert points[0] == origin
        points[0] = target if starting_from_target else origin

        for i in range(1, len(points)):
            direction = norm(points[i] - points[i - 1])
            points[i] = points[i - 1] + direction * segment_lengths[i - 1]

        distance_to_target = vlen(points[-1] - target)
        if not starting_from_target and distance_to_target <= min_acceptable_distance:
            return


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
