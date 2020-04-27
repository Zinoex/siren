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


def queue_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon):

    queue = init.queue
    queue_objective = gp.LinExpr()
    stops_objective = gp.LinExpr()

    for k in range(1, prediction_horizon + 1):
        for s in range(num_signals):
            # TODO: Make sure queue doesn't go negative
            # qnegative = m.addVar(vtype=GRB.INTEGER, 'qnegative_{}_{}'.format(k, s))
            # m.addConstr(qnegative <= 0)
            # qpositive = m.addVar(vtype=GRB.INTEGER, 'qpositive_{}_{}'.format(k, s))
            # m.addConstr(qpositive >= 0)

            #m.addConstr(queue[s] + arr_fn(k, s) - dep_fn(k, s) * (colors[k, s, GREEN] + colors[k, s, YELLOW]) + qnegative + qpositive == 0, 'qresidual_{}_{}'.format(k, s))

            queue[s] += arr_fn(k, s) - dep_fn(k, s) * (colors[k, s, GREEN] + colors[k, s, YELLOW])

            # Objective functions update
            queue_objective.add(queue[s])  # Queue
            stops_objective.add(arr_fn(k, s) * (colors[k, s, RED] + colors[k, s, YELLOW]))  # Stops

    return queue_objective, stops_objective


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


def min_green_constraints(m, colors, init, min_green, prediction_horizon, num_signals):
    tg = init.timing.green

    for k in range(1, prediction_horizon + 1):
        for s in range(num_signals):
            # Constrain min green
            now = (1 - colors[k, s, GREEN])
            before = colors[k - 1, s, GREEN]
            tg_product = logical_product(m, now, before, name='tg_product_{}_{}'.format(k, s))

            m.addConstr(min_green[s] * tg_product - tg[s] <= 0, 'min_green_{}_{}'.format(k, s))

            # Update tg
            tg_aux = m.addVar(vtype=GRB.INTEGER, name='tg_aux_{}_{}'.format(k, s))
            # m.addConstr(tg_aux <= init.timing.green[s] + prediction_horizon)
            m.addConstr(tg_aux == tg[s] * (1 - colors[k, s, GREEN]), 'tg_aux_constr_{}_{}'.format(k, s))

            tg[s] += colors[k, s, GREEN] - tg_aux


def amber_time_constraints(m, colors, init, amber_time, prediction_horizon, num_signals):
    ta = init.timing.amber
    ma = max(amber_time)

    for k in range(1, prediction_horizon + 1):
        for s in range(num_signals):
            # Constrain amber time
            now = (1 - colors[k, s, AMBER])
            before = colors[k - 1, s, AMBER]
            ta_product = logical_product(m, now, before, name='ta_product_{}_{}'.format(k, s))

            m.addConstr((amber_time[s] - ta[s]) * ta_product == 0, 'amber_time_{}_{}'.format(k, s))

            # Update ta
            ta_aux = m.addVar(vtype=GRB.INTEGER, name='ta_aux_{}_{}'.format(k, s))
            # m.addConstr(ta_aux <= ma)
            m.addConstr(ta_aux == ta[s] * now, 'ta_aux_constr_{}_{}'.format(k, s))

            ta[s] += colors[k, s, AMBER] - ta_aux


def yellow_time_constraints(m, colors, init, yellow, prediction_horizon, num_signals):
    ty = init.timing.yellow
    my = max(yellow)

    for k in range(1, prediction_horizon + 1):
        for s in range(num_signals):
            # Constrain amber time
            now = (1 - colors[k, s, YELLOW])
            before = colors[k - 1, s, YELLOW]
            ty_product = logical_product(m, now, before, name='ty_product_{}_{}'.format(k, s))

            m.addConstr((yellow[s] - ty[s]) * ty_product == 0, 'yellow_time_{}_{}'.format(k, s))

            # Update ta
            ty_aux = m.addVar(vtype=GRB.INTEGER, name='ty_aux_{}_{}'.format(k, s))
            # m.addConstr(ty_aux <= my)
            m.addConstr(ty_aux == ty[s] * now, 'ty_aux_constr_{}_{}'.format(k, s))

            ty[s] += colors[k, s, YELLOW] - ty_aux


def logical_product(m, x1, x2, name=''):
    product = m.addVar(vtype=GRB.BINARY, name=name)
    m.addConstr(product == x1 * x2)

    return product


def green_interval(m, colors, init, green_interval_matrix, prediction_horizon, num_signals):
    tng = init.timing.not_green

    for k in range(1, prediction_horizon + 1):
        for s1 in range(num_signals):
            for s2 in range(num_signals):
                # Constrain green interval
                # now = colors[k, s1, GREEN]
                # before = (1 - colors[k - 1, s1, GREEN])
                # tng_product = logical_product(m, now, before, name='tng_product_{}_{}_{}'.format(k, s1, s2))

                m.addConstr((green_interval_matrix[s2, s1] - tng[s2]) * colors[k, s1, GREEN] <= 0, 'green_interval_{}_{}_{}'.format(k, s1, s2))

        for s in range(num_signals):
            # Update tng
            tng_aux = m.addVar(vtype=GRB.INTEGER, name='tng_aux_{}_{}'.format(k, s))
            m.addConstr(tng_aux <= init.timing.not_green[s] + prediction_horizon)
            m.addConstr(tng_aux == tng[s] * colors[k, s, GREEN], 'tng_aux_constr_{}_{}'.format(k, s))

            tng[s] += (1 - colors[k, s, GREEN]) - tng_aux


def create_model(intersection, init, arr_fn, dep_fn, prediction_horizon=30):
    # Create a new model
    m = gp.Model('siren')
    num_signals = intersection.num_signals

    # Create light variables
    colors = np.empty((prediction_horizon + 1, num_signals, num_colors), dtype=object)
    for k in range(prediction_horizon + 1):
        for s in range(num_signals):
            for c in range(num_colors):
                colors[k, s, c] = m.addVar(vtype=GRB.BINARY, name='{}_{}_{}'.format(color_name[c], k, s))

    # Add system dynamics
    queue_objective, stops_objective = queue_evolution(m, colors, init, arr_fn, dep_fn, num_signals, prediction_horizon)

    # Add constraints
    initial_light_constraints(m, colors, init, num_signals)
    single_light_constraints(m, colors, prediction_horizon, num_signals)
    conflict_constraints(m, colors, intersection.conflict_matrix, prediction_horizon, num_signals)
    green_transition_constraints(m, colors, intersection.yellow_time, prediction_horizon, num_signals)
    red_transition_constraints(m, colors, intersection.amber_time, prediction_horizon, num_signals)
    yellow_transition_constraints(m, colors, prediction_horizon, num_signals)
    amber_transition_constraints(m, colors, prediction_horizon, num_signals)
    min_green_constraints(m, colors, init, intersection.min_green, prediction_horizon, num_signals)
    amber_time_constraints(m, colors, init, intersection.amber_time, prediction_horizon, num_signals)
    yellow_time_constraints(m, colors, init, intersection.yellow_time, prediction_horizon, num_signals)
    green_interval(m, colors, init, intersection.green_interval, prediction_horizon, num_signals)

    # Set object function
    m.setObjectiveN(queue_objective, 0)
    m.setObjectiveN(stops_objective, 1)

    return m
