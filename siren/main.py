from gurobi.model import Intersection
from gurobi.intersections import super_simple_init as init, super_simple_arrival_function as arrival_function, super_simple_departure_function as departure_function, super_simple_configuration as configuration
from gurobi.structures import Options

import timeit


def test():
    model = Intersection(configuration, Options(), arrival_function, departure_function, init)
    model.optimize()


print(timeit.timeit(test, number=10))

# model = Intersection(configuration, Options(), arrival_function, departure_function, init)
# model.optimize()
# colors = model.get_colors()
# print(colors)
