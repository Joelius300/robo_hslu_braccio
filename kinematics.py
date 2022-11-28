import math
from typing import Literal

import numpy as np


def rot(axis: Literal['x'] | Literal['y'] | Literal['z'], angle: int | float):
    pass  # TODO


def trans(x: int | float, y: int | float, z: int | float):
    pass  # TODO


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
