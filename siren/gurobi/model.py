import gurobipy as gp
from gurobipy import GRB

import numpy as np


class Intersection:
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

    def __init__(self, configuration, options, arr_fn, dep_fn, init):
        # Create a new model
        self.model = gp.Model('siren')
        self.configuration = configuration
        self.options = options

        # Create light variables
        self.colors = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals, self.num_colors),
                               dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.colors[k, s, c] = self.model.addVar(vtype=GRB.BINARY,
                                                             name='{}_{}_{}'.format(self.color_name[c], k, s))

        # Add system dynamics
        self.initial_queue = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_queue[s] = gp.LinExpr(
                init.queue[s])
        queue_objective = self.queue_evolution(arr_fn, dep_fn)
        stops_objective = self.stops_evolution(arr_fn)

        # Add constraints
        self.single_light_constraints()
        self.conflict_constraints()
        self.green_transition_constraints()
        self.red_transition_constraints()
        self.yellow_transition_constraints()
        self.amber_transition_constraints()

        # Add initial timing dependent constraints
        self.initial_green = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_green[s] = gp.LinExpr(init.timing.green[s])
        self.min_green_constraints()

        self.initial_amber = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_amber[s] = gp.LinExpr(init.timing.amber[s])
        self.amber_time_constraints()

        self.initial_yellow = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_yellow[s] = gp.LinExpr(init.timing.yellow[s])
        self.yellow_time_constraints()

        self.initial_notgreen = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_notgreen[s] = gp.LinExpr(
                init.timing.not_green[s])
        self.green_interval()

        self.initial_light_constraints(init)

        # Set object function
        self.model.setObjective(
            self.options.queue_weight * queue_objective +
            self.options.stops_weight * stops_objective,
            GRB.MINIMIZE)

    def optimize(self):
        self.model.optimize()

        for v in self.model.getVars():
            print('%s %g' % (v.varName, v.x))

        print('Obj: %g' % self.model.objVal)

    #####################
    # System dynamics
    #####################
    def queue_evolution(self, arr_fn, dep_fn):
        queue = self.initial_queue
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                q_res = queue[s] + arr_fn(k, s) - dep_fn(k, s) * (
                            self.colors[k, s, self.GREEN] + self.colors[k, s, self.YELLOW])

                q = self.model.addVar(vtype=GRB.INTEGER, name='q_{}_{}'.format(k, s))
                self.model.addGenConstrIndicator(q, False, q_res <= 0, name='q_residual_{}_{}'.format(k, s))
                q_prime = self.model.addVar(vtype=GRB.INTEGER, name='q_prime_{}_{}'.format(k, s))
                self.model.addConstr(q_prime == q * q_res)

                queue[s] = q_prime

                # Objective functions update
                objective.add(queue[s])  # Queue

        return objective

    def stops_evolution(self, arr_fn):
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                objective.add(arr_fn(k, s) * (self.colors[k, s, self.RED] + self.colors[k, s, self.AMBER]))  # Stops

        return objective

    # def wait_time_evolution(self, arr_fn, dep_fn):
    #     queue = self.initial_queue
    #     objective = gp.LinExpr()
    #
    #     for k in range(1, self.options.prediction_horizon + 1):
    #         for s in range(self.configuration.num_signals):
    #             q_res = queue[s] + arr_fn(k, s) - dep_fn(k, s) * (
    #                         self.colors[k, s, self.GREEN] + self.colors[k, s, self.YELLOW])
    #
    #             q = self.model.addVar(vtype=GRB.INTEGER, name='q_{}_{}'.format(k, s))
    #             self.model.addGenConstrIndicator(q, False, q_res <= 0, name='q_residual_{}_{}'.format(k, s))
    #             q_prime = self.model.addVar(vtype=GRB.INTEGER, name='q_prime_{}_{}'.format(k, s))
    #             self.model.addConstr(q_prime == q * q_res)
    #
    #             queue[s] = q_prime
    #
    #             # Objective functions update
    #             objective.add(queue[s])  # Queue
    #
    #     return objective

    #########################
    # General constraints
    #########################
    def single_light_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(self.colors[k, s, :].sum() == 1, 'color_sum_{}_{}'.format(k, s))

    def nonblocking(self, k, s):
        return self.colors[k, s, self.GREEN] + self.colors[k, s, self.YELLOW] + self.colors[k, s, self.AMBER]

    def conflict_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s1 in range(self.configuration.num_signals):
                for s2 in range(self.configuration.num_signals):
                    self.model.addConstr(self.configuration.conflict_matrix[s1, s2] * (
                                self.nonblocking(k, s1) + self.nonblocking(k, s2)) <= 1,
                                         'conflict_{}_{}_{}'.format(k, s1, s2))

    def stable_light_transition_constraint(self, stable, after_not, after_zero, after_positive, timing):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, stable] + self.colors[k, s, after_not] <= 1,
                                     'trans_{}_{}_{}_{}'.format(stable, after_not, k, s))
                self.model.addConstr(
                    (self.colors[k - 1, s, stable] + self.colors[k, s, after_zero]) * timing[s] <= 1 + timing[s],
                    'trans_{}_{}_{}_{}'.format(stable, after_zero, k, s))
                self.model.addConstr(
                    self.colors[k - 1, s, stable] + self.colors[k, s, after_positive] * (1 - timing[s]) <= 1,
                    'trans_{}_{}_{}_{}'.format(stable, after_positive, k, s))

    def green_transition_constraints(self):
        self.stable_light_transition_constraint(self.GREEN, self.AMBER, self.RED, self.YELLOW,
                                                self.configuration.yellow_time)

    def red_transition_constraints(self):
        self.stable_light_transition_constraint(self.RED, self.YELLOW, self.GREEN, self.AMBER,
                                                self.configuration.amber_time)

    def intermediate_transition_constraints(self, intermediate, after1, after2):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after1] <= 1,
                                     'trans_{}_{}_{}_{}'.format(intermediate, after1, k, s))
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after2] <= 1,
                                     'trans_{}_{}_{}_{}'.format(intermediate, after2, k, s))

    def yellow_transition_constraints(self):
        self.intermediate_transition_constraints(self.YELLOW, self.GREEN, self.AMBER)

    def amber_transition_constraints(self):
        self.intermediate_transition_constraints(self.AMBER, self.YELLOW, self.RED)

    #########################################
    # Initial timing dependent constraints
    #########################################
    def min_green_constraints(self):
        tg = self.initial_green

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain min green
                now = (1 - self.colors[k, s, self.GREEN])
                before = self.colors[k - 1, s, self.GREEN]
                self.model.addConstr(
                    self.configuration.min_green[s] * (now + before - 1) <= tg[s],
                    'min_green_{}_{}'.format(k, s))

                # Update tg
                tg_aux = self.model.addVar(vtype=GRB.INTEGER, name='tg_aux_{}_{}'.format(k, s))
                self.model.addConstr(tg_aux <= self.initial_green[s] + self.options.prediction_horizon)
                self.model.addConstr(tg_aux >= 0)
                self.model.addConstr(tg_aux == tg[s] * (1 - self.colors[k, s, self.GREEN]),
                                     'tg_aux_constr_{}_{}'.format(k, s))

                tg[s] += self.colors[k, s, self.GREEN] - tg_aux

    def amber_time_constraints(self):
        ta = self.initial_amber
        ma = max(self.configuration.amber_time)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain amber time
                now = (1 - self.colors[k, s, self.AMBER])
                before = self.colors[k - 1, s, self.AMBER]
                ta_product = self.logical_product(now, before, name='ta_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.amber_time[s] - ta[s]) * ta_product == 0,
                                     'amber_time_max_{}_{}'.format(k, s))

                # Update ta
                ta_aux = self.model.addVar(vtype=GRB.INTEGER, name='ta_aux_{}_{}'.format(k, s))
                self.model.addConstr(ta_aux <= ma)
                self.model.addConstr(ta_aux >= 0)
                self.model.addConstr(ta_aux == ta[s] * now, 'ta_aux_constr_{}_{}'.format(k, s))

                ta[s] += self.colors[k, s, self.AMBER] - ta_aux

                self.model.addConstr(self.configuration.amber_time[s] * self.colors[k, s, self.AMBER] >= ta[s],
                                     'amber_time_min_{}_{}'.format(k, s))

    def yellow_time_constraints(self):
        ty = self.initial_yellow
        my = max(self.configuration.yellow_time)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain amber time
                now = (1 - self.colors[k, s, self.YELLOW])
                before = self.colors[k - 1, s, self.YELLOW]
                ty_product = self.logical_product(now, before, name='ty_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.yellow_time[s] - ty[s]) * ty_product == 0,
                                     'yellow_time_max_{}_{}'.format(k, s))

                # Update ta
                ty_aux = self.model.addVar(vtype=GRB.INTEGER, name='ty_aux_{}_{}'.format(k, s))
                self.model.addConstr(ty_aux <= my)
                self.model.addConstr(ty_aux >= 0)
                self.model.addConstr(ty_aux == ty[s] * now, 'ty_aux_constr_{}_{}'.format(k, s))

                ty[s] += self.colors[k, s, self.YELLOW] - ty_aux

                self.model.addConstr(self.configuration.yellow_time[s] * self.colors[k, s, self.YELLOW] >= ty[s],
                                     'yellow_time_min_{}_{}'.format(k, s))

    def logical_product(self, x1, x2, name=''):
        product = self.model.addVar(vtype=GRB.BINARY, name=name)
        self.model.addConstr(product == x1 * x2)

        return product

    def green_interval(self):
        tng = self.initial_notgreen

        for k in range(1, self.options.prediction_horizon + 1):
            for s1 in range(self.configuration.num_signals):
                for s2 in range(self.configuration.num_signals):
                    # Constrain green interval
                    self.model.addConstr(
                        (self.configuration.green_interval[s2, s1] - tng[s2]) * self.colors[k, s1, self.GREEN] <= 0,
                        'green_interval_{}_{}_{}'.format(k, s1, s2))

            for s in range(self.configuration.num_signals):
                # Update tng
                tng_aux = self.model.addVar(vtype=GRB.INTEGER, name='tng_aux_{}_{}'.format(k, s))
                self.model.addConstr(tng_aux <= self.initial_notgreen[s] + self.options.prediction_horizon)
                self.model.addConstr(tng_aux >= 0)
                self.model.addConstr(tng_aux == tng[s] * self.colors[k, s, self.GREEN],
                                     'tng_aux_constr_{}_{}'.format(k, s))

                tng[s] += (1 - self.colors[k, s, self.GREEN]) - tng_aux

    ##########################
    # Initial constraints
    ##########################
    def initial_light_constraints(self, init):
        for s in range(self.configuration.num_signals):
            for c in range(self.num_colors):
                self.model.addConstr(self.colors[0, s, c] == init.lights[s, c],
                                     'init_light_{}_{}'.format(self.color_name[c], s))
