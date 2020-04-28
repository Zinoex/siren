from .structures import Initialization, Configuration, Timing

import numpy as np


def super_simple_arrival_function(k, s):
    return 1


def super_simple_departure_function(k, s):
    return 3


super_simple_init = Initialization(
    lights=np.array([[0, 1, 0, 0], [0, 1, 0, 0]]),
    timing=Timing(
        np.array([0, 0]),
        np.array([0, 0]),
        np.array([0, 0]),
        np.array([9, 2]),
        np.array([2, 0])
    ),
    queue=np.array([1, 0])
)

super_simple_configuration = Configuration(
    2,
    np.array([[0, 1], [1, 0]]),
    np.array([[0, 10], [10, 0]]),
    np.array([4, 4]),
    np.array([2, 2]),
    np.array([6, 6])
)
