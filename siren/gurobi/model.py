import gurobipy as gp
from gurobipy import GRB

import numpy as np


num_colors = 4

GREEN = 0
RED = 1
YELLOW = 2
AMBER = 3

color_name = {
    GREEN: 'green',
    RED: 'red',
    YELLOW: 'yellow',
    AMBER: 'amber'
}


def add_initial_light_constraint(m, colors, init, num_signals):
    for s in range(num_signals):
        for c in range(num_colors):
            m.addConstr(colors[0, s, c] == init.lights[s, c], 'init_light_{}_{}'.format(color_name[c], s))


def add_single_light_constraint(m, colors, prediction_horizon, num_signals):
    for k in range(prediction_horizon + 1):
        for s in range(num_signals):
            m.addConstr(colors[k, s, :].sum() == 1, 'color_sum_{}_{}'.format(k, s))


def add_state_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon):

    # tg = init.timings.green
    # tr = init.timings.red
    # ty = init.timings.yellow
    # ta = init.timings.amber
    # tng = init.timings.not_green

    queue = init.queue
    objective = gp.LinExpr()

    for k in range(1, prediction_horizon + 1):
        for s in range(num_signals):
            # TODO: Make sure queue doesn't go negative
            queue[s] += arr_fn(k, s) - dep_fn(k, s) * (colors[k, s, GREEN] + colors[k, s, YELLOW])
            objective.add(queue[s])

    return objective


def create_model(init, arr_fn, dep_fn, num_signals=2, prediction_horizon=30):
    # Create a new model
    m = gp.Model("siren")

    # Create light variables
    colors = np.empty((prediction_horizon + 1, num_signals, num_colors), dtype=object)
    for k in range(prediction_horizon + 1):
        for s in range(num_signals):
            for c in range(num_colors):
                colors[k, s, c] = m.addVar(vtype=GRB.BINARY, name="{}_{}_{}".format(color_name[c], k, s))

    # Add system dynamics
    objective = add_state_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon)

    # Add constraints
    add_initial_light_constraint(m, colors, init, num_signals)
    add_single_light_constraint(m, colors, prediction_horizon, num_signals)

    # Set object function
    m.setObjective(objective, GRB.MINIMIZE)

    return m


class Init:
    def __init__(self, lights, timing, queue):
        self.lights = lights
        self.timing = timing
        self.queue = queue


class Timing:
    def __init__(self, green, red, yellow, amber, not_green):
        self.green = green
        self.red = red
        self.yellow = yellow
        self.amber = amber
        self.not_green = not_green


def arrival_function(k, s):
    return 1


def departure_function(k, s):
    return 1


initialization = Init(
    np.array([[0, 1, 0, 0], [0, 1, 0, 0]]),
    None,
    np.array([gp.LinExpr(), gp.LinExpr()])
)

model = create_model(initialization, arrival_function, departure_function)
model.optimize()

for v in model.getVars():
    print('%s %g' % (v.varName, v.x))

print('Obj: %g' % model.objVal)
