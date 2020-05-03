from gurobi.structures import Configuration

import numpy as np


class SuperSimpleArrival:
    def __getitem__(self, item):
        return 1


class SuperSimpleDeparture:
    def __getitem__(self, item):
        return 3


super_simple_configuration = Configuration(
    2,
    np.array([[0, 1], [1, 0]]),
    np.array([[0, 10], [10, 0]]),
    np.array([4, 4]),
    np.array([2, 2]),
    np.array([6, 6])
)
