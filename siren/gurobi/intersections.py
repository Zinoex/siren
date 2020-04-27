
from .structures import Init, Intersection

import gurobipy as gp

import numpy as np


def super_simple_arrival_function(k, s):
    return 1


def super_simple_departure_function(k, s):
    return 1


super_simple_init = Init(
    np.array([[0, 1, 0, 0], [0, 1, 0, 0]]),
    None,
    np.array([gp.LinExpr(), gp.LinExpr()])
)

super_simple_intersection = Intersection(
    2,
    np.array([[0, 1], [1, 0]]),
    np.array([4, 4]),
    np.array([2, 2])
)
