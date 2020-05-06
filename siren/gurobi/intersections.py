from gurobi.structures import GurobiConfiguration

import numpy as np


super_simple_configuration = GurobiConfiguration(
    2,
    np.array([[0, 1], [1, 0]]),
    np.array([[0, 10], [10, 0]]),
    np.array([4, 4]),
    np.array([2, 2]),
    np.array([6, 6]),
    np.array([300, 300])
)
