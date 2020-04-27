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


def state_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon):

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
            qnegative = m.addVar(vtype=GRB.INTEGER, name="qnegative_{}_{}".format(k, s))
            m.addConstr(qnegative <= 0)
            qpositive = m.addVar(vtype=GRB.INTEGER, name="qpositive_{}_{}".format(k, s))
            m.addConstr(qpositive >= 0)

            #m.addConstr(queue[s] + arr_fn(k, s) - dep_fn(k, s) * (colors[k, s, GREEN] + colors[k, s, YELLOW]) + qnegative + qpositive == 0, name='qresidual_{}_{}'.format(k, s))

            queue[s] += arr_fn(k, s) - dep_fn(k, s) * (colors[k, s, GREEN] + colors[k, s, YELLOW])

            # Objective function update
            objective.add(queue[s])
            objective.add(arr_fn(k, s) * (colors[k, s, RED] + colors[k, s, YELLOW]))

    return objective


def initial_light_constraints(m, colors, init, num_signals):
    for s in range(num_signals):
        for c in range(num_colors):
            m.addConstr(colors[0, s, c] == init.lights[s, c], 'init_light_{}_{}'.format(color_name[c], s))


def single_light_constraints(m, colors, prediction_horizon, num_signals):
    for k in range(prediction_horizon + 1):
        for s in range(num_signals):
            m.addConstr(colors[k, s, :].sum() == 1, 'color_sum_{}_{}'.format(k, s))


def conflict_constraints(m, colors, conflict_matrix, prediction_horizon, num_signals):
    for k in range(prediction_horizon + 1):
        for s1 in range(num_signals):
            for s2 in range(num_signals):
                nonblocking1 = colors[k, s1, GREEN] + colors[k, s1, YELLOW] + colors[k, s1, AMBER]
                nonblocking2 = colors[k, s2, GREEN] + colors[k, s2, YELLOW] + colors[k, s2, AMBER]
                field = conflict_matrix[s1, s2]

                m.addConstr(field * (nonblocking1 + nonblocking2) <= 1, 'conflict_{}_{}_{}'.format(k, s1, s2))


def green_transition_constraints(m, colors, yellow_time, prediction_horizon, num_signals):
    for s in range(num_signals):
        for k in range(1, prediction_horizon + 1):
            m.addConstr(colors[k - 1, s, GREEN] + colors[k, s, AMBER] <= 1, 'trans_green_amber_{}_{}'.format(k, s))
            m.addConstr((colors[k - 1, s, GREEN] + colors[k, s, RED]) * yellow_time[s] <= 1 + yellow_time[s], 'trans_green_red_{}_{}'.format(k, s))
            m.addConstr(colors[k - 1, s, GREEN] + colors[k, s, YELLOW] * (1 - yellow_time[s]) <= 1, 'trans_green_yellow_{}_{}'.format(k, s))


def red_transition_constraints(m, colors, amber_time, prediction_horizon, num_signals):
    for s in range(num_signals):
        for k in range(1, prediction_horizon + 1):
            m.addConstr(colors[k - 1, s, RED] + colors[k, s, YELLOW] <= 1, 'trans_red_yellow_{}_{}'.format(k, s))
            m.addConstr((colors[k - 1, s, RED] + colors[k, s, GREEN]) * amber_time[s] <= 1 + amber_time[s], 'trans_red_green_{}_{}'.format(k, s))
            m.addConstr((colors[k - 1, s, RED] + colors[k, s, YELLOW]) * (1 - amber_time[s]) <= 1, 'trans_red_amber_{}_{}'.format(k, s))


def yellow_transition_constraints(m, colors, prediction_horizon, num_signals):
    for s in range(num_signals):
        for k in range(1, prediction_horizon + 1):
            m.addConstr(colors[k - 1, s, YELLOW] + colors[k, s, GREEN] <= 1, 'trans_yellow_green_{}_{}'.format(k, s))
            m.addConstr(colors[k - 1, s, YELLOW] + colors[k, s, AMBER] <= 1, 'trans_yellow_amber_{}_{}'.format(k, s))


def amber_transition_constraints(m, colors, prediction_horizon, num_signals):
    for s in range(num_signals):
        for k in range(1, prediction_horizon + 1):
            m.addConstr(colors[k - 1, s, AMBER] + colors[k, s, YELLOW] <= 1, 'trans_amber_yellow_{}_{}'.format(k, s))
            m.addConstr(colors[k - 1, s, AMBER] + colors[k, s, RED] <= 1, 'trans_amber_red_{}_{}'.format(k, s))


def create_model(intersection, init, arr_fn, dep_fn, prediction_horizon=60):
    # Create a new model
    m = gp.Model("siren")
    num_signals = intersection.num_signals

    # Create light variables
    colors = np.empty((prediction_horizon + 1, num_signals, num_colors), dtype=object)
    for k in range(prediction_horizon + 1):
        for s in range(num_signals):
            for c in range(num_colors):
                colors[k, s, c] = m.addVar(vtype=GRB.BINARY, name="{}_{}_{}".format(color_name[c], k, s))

    # Add system dynamics
    objective = state_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon)

    # Add constraints
    initial_light_constraints(m, colors, init, num_signals)
    single_light_constraints(m, colors, prediction_horizon, num_signals)
    conflict_constraints(m, colors, intersection.conflict_matrix, prediction_horizon, num_signals)
    green_transition_constraints(m, colors, intersection.yellow_time, prediction_horizon, num_signals)
    red_transition_constraints(m, colors, intersection.amber_time, prediction_horizon, num_signals)
    yellow_transition_constraints(m, colors, prediction_horizon, num_signals)
    amber_transition_constraints(m, colors, prediction_horizon, num_signals)

    # Set object function
    m.setObjective(objective, GRB.MINIMIZE)

    return m
