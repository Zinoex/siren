from gurobi.model import create_model
from gurobi.intersections import super_simple_init as init, super_simple_arrival_function as arrival_function, super_simple_departure_function as departure_function, super_simple_intersection as intersection

import timeit

model = create_model(intersection, init, arrival_function, departure_function)

#print(timeit.timeit(model.optimize, number=100))
model.optimize()

for v in model.getVars():
    print('%s %g' % (v.varName, v.x))

print('Obj: %g' % model.objVal)
