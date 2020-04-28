from gurobi.model import Intersection
from gurobi.intersections import super_simple_init as init, super_simple_arrival_function as arrival_function, super_simple_departure_function as departure_function, super_simple_configuration as configuration
from gurobi.structures import Options

import timeit

model = Intersection(configuration, Options(), arrival_function, departure_function, init)


print(timeit.timeit(model.optimize, number=1))
# model.optimize()
