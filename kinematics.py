from __future__ import annotations

import math
from typing import Literal

import numpy as np


def rot(axis: Literal['x', 'y', 'z'], angle: int | float):
    """Create a rotation matrix for angle around axis"""

    t = angle * np.pi / 180
    c = cos(t)
    s = sin(t)
    m = np.array()

    if axis == 'x':
        m = np.array([
            [1, 0, 0, 0],
            [0, c, -s, 0],
            [0, s, c, 0],
            [0, 0, 0, 1]
        ])
    elif axis == 'y':
        m = np.array([
            [c, 0, s, 0],
            [0, 1, 0, 0],
            [-s, 0, c, 0],
            [0, 0, 0, 1]
        ])
    else:
        m = np.array([
            [c, -s, 0, 0],
            [s, c, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ])
    return m


def trans(x=0, y=0, z=0):
    """Create a translation matrix for x, y, z"""
    return np.array([
        [1, 0, 0, x],
        [0, 1, 0, y],
        [0, 0, 1, z],
        [0, 0, 0, 1]
    ])


def arr(*args):
    return np.array(args)


def vec(*args):
    return np.array([[k] for k in args])


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
