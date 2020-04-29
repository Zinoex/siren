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
        stops_objective = self.stops_evolution(arr_fn)

        self.initial_queue = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_queue[s] = gp.LinExpr(init.queue[s])
        queue_objective, queue_notempty = self.queue_evolution(arr_fn, dep_fn)

        self.initial_wait_time = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_wait_time[s] = gp.LinExpr(init.timing.wait[s])
        wait_objective = self.wait_time_evolution(queue_notempty)

        # Add constraints
        self.single_light_constraints()
        self.conflict_constraints()
        self.green_transition_constraints()
        self.red_transition_constraints()
        self.yellow_transition_constraints()
        self.amber_transition_constraints()
        self.control_horizon_constraints()

        # Add initial timing dependent constraints
        self.initial_green = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_green[s] = gp.LinExpr(init.timing.green[s])
        tg_history = self.min_green_constraints()

        self.initial_amber = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_amber[s] = gp.LinExpr(init.timing.amber[s])
        ta_history = self.amber_time_constraints()

        self.initial_yellow = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_yellow[s] = gp.LinExpr(init.timing.yellow[s])
        ty_history = self.yellow_time_constraints()

        self.initial_notgreen = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_notgreen[s] = gp.LinExpr(init.timing.not_green[s])
        tng_history = self.green_interval()

        self.initial_light_constraints(init)

        # Additional constraints to speed up the execution
        for k in range(self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(ta_history[k, s] + ty_history[k, s] <= tng_history[k, s])

        # Set object function
        self.model.setObjective(
            self.options.queue_weight * queue_objective +
            self.options.stops_weight * stops_objective +
            self.options.wait_weight * wait_objective,
            GRB.MINIMIZE)

        # Reduces execution time by 40%
        self.model.Params.PreDual = 1
        self.model.Params.Quad = 0
        self.model.Params.MIPGap = 0.1
        self.model.Params.TimeLimit = 2

    def optimize(self, verbose=False):
        # self.model.tune()
        #
        # if self.model.tuneResultCount > 0:
        #     # Load the best tuned parameters into the model
        #     self.model.getTuneResult(0)

        self.model.optimize()

        if verbose:
            for v in self.model.getVars():
                print('%s %g' % (v.varName, v.x))

            print('Obj: %g' % self.model.objVal)

    def get_colors(self, k=1):
        def select_value(color):
            return color.x

        return np.vectorize(select_value)(self.colors[k, :, :])

    #####################
    # System dynamics
    #####################
    def queue_evolution(self, arr_fn, dep_fn):
        queue = self.initial_queue
        objective = gp.LinExpr()

        queue_notempty = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                q_res = queue[s] + arr_fn(k, s) - dep_fn(k, s) * (
                            self.colors[k, s, self.GREEN] + self.colors[k, s, self.YELLOW])

                q = self.model.addVar(vtype=GRB.BINARY, name='q_{}_{}'.format(k, s))
                self.model.addGenConstrIndicator(q, False, q_res <= 0, name='q_residual_{}_{}'.format(k, s))
                q_prime = self.model.addVar(vtype=GRB.INTEGER, name='q_prime_{}_{}'.format(k, s))
                self.model.addConstr(q_prime == q * q_res)

                queue[s] = q_prime
                queue_notempty[k - 1, s] = q

                # Update objective function
                objective.add(queue[s])

        return objective, queue_notempty

    def stops_evolution(self, arr_fn):
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                objective.add(arr_fn(k, s) * (self.colors[k, s, self.RED] + self.colors[k, s, self.AMBER]))

        return objective

    def wait_time_evolution(self, queue_notempty):
        tq = self.initial_wait_time
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                q_notempty = queue_notempty[k - 1, s]

                request = self.logical_product(q_notempty, (1 - self.colors[k, s, self.GREEN]), name='request_{}_{}'.format(k, s))

                tq_aux = self.model.addVar(vtype=GRB.INTEGER, name='tq_aux_{}_{}'.format(k, s))
                self.model.addConstr(tq_aux <= self.initial_wait_time[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tq_aux >= 0)
                self.model.addConstr(tq_aux == tq[s] * (1 - request), 'tq_aux_constr_{}_{}'.format(k, s))

                tq[s] += request - tq_aux
                self.model.addConstr(tq[s] <= self.initial_wait_time[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tq[s] >= 0)

                # Update objective function
                objective.add(tq[s])

        return objective

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

    def control_horizon_constraints(self):
        for k in range(self.options.control_horizon + 1, self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.model.addConstr(self.colors[k, s, c] == self.colors[k + 1, s, c],
                                         'control_horizon_{}_{}_{}'.format(k, s, c))

    #########################################
    # Initial timing dependent constraints
    #########################################
    def min_green_constraints(self):
        tg = self.initial_green
        tg_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain min green
                self.model.addConstr(
                    self.configuration.min_green[s] * (self.colors[k - 1, s, self.GREEN] - self.colors[k, s, self.GREEN]) <= tg[s],
                    'min_green_{}_{}'.format(k, s))

                # Update tg
                tg_aux = self.model.addVar(vtype=GRB.INTEGER, name='tg_aux_{}_{}'.format(k, s))
                self.model.addConstr(tg_aux <= self.initial_green[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tg_aux >= 0)
                self.model.addConstr(tg_aux == tg[s] * (1 - self.colors[k, s, self.GREEN]),
                                     'tg_aux_constr_{}_{}'.format(k, s))

                tg[s] += self.colors[k, s, self.GREEN] - tg_aux
                self.model.addConstr(tg[s] <= self.initial_green[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tg[s] >= 0)

                tg_history[k - 1, s] = tg[s]

        return tg_history

    def amber_time_constraints(self):
        ta = self.initial_amber
        ma = max(self.configuration.amber_time)

        ta_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain amber time
                ta_product = self.logical_product((1 - self.colors[k, s, self.AMBER]), self.colors[k - 1, s, self.AMBER], name='ta_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.amber_time[s] - ta[s]) * ta_product == 0,
                                     'amber_time_max_{}_{}'.format(k, s))

                # Update ta
                ta_aux = self.model.addVar(vtype=GRB.INTEGER, name='ta_aux_{}_{}'.format(k, s))
                self.model.addConstr(ta_aux <= ma)
                self.model.addConstr(ta_aux >= 0)
                self.model.addConstr(ta_aux == ta[s] * (1 - self.colors[k, s, self.AMBER]), 'ta_aux_constr_{}_{}'.format(k, s))

                ta[s] += self.colors[k, s, self.AMBER] - ta_aux
                self.model.addConstr(ta[s] <= ma)
                self.model.addConstr(ta[s] >= 0)

                self.model.addConstr(self.configuration.amber_time[s] * self.colors[k, s, self.AMBER] >= ta[s],
                                     'amber_time_min_{}_{}'.format(k, s))

                ta_history[k - 1, s] = ta[s]

        return ta_history

    def yellow_time_constraints(self):
        ty = self.initial_yellow
        my = max(self.configuration.yellow_time)

        ty_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                # Constrain amber time
                ty_product = self.logical_product((1 - self.colors[k, s, self.YELLOW]), self.colors[k - 1, s, self.YELLOW], name='ty_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.yellow_time[s] - ty[s]) * ty_product == 0,
                                     'yellow_time_max_{}_{}'.format(k, s))

                # Update ta
                ty_aux = self.model.addVar(vtype=GRB.INTEGER, name='ty_aux_{}_{}'.format(k, s))
                self.model.addConstr(ty_aux <= my)
                self.model.addConstr(ty_aux >= 0)
                self.model.addConstr(ty_aux == ty[s] * (1 - self.colors[k, s, self.YELLOW]), 'ty_aux_constr_{}_{}'.format(k, s))

                ty[s] += self.colors[k, s, self.YELLOW] - ty_aux
                self.model.addConstr(ty[s] <= my)
                self.model.addConstr(ty[s] >= 0)

                self.model.addConstr(self.configuration.yellow_time[s] * self.colors[k, s, self.YELLOW] >= ty[s],
                                     'yellow_time_min_{}_{}'.format(k, s))

                ty_history[k - 1, s] = ty[s]

        return ty_history

    def logical_product(self, x1, x2, name=''):
        product = self.model.addVar(vtype=GRB.BINARY, name=name)
        self.model.addConstr(product - x1 <= 0)
        self.model.addConstr(product - x2 <= 0)
        self.model.addConstr(x1 + x2 - product <= 1)

        return product

    def green_interval(self):
        tng = self.initial_notgreen

        tng_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

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
                self.model.addConstr(tng_aux <= self.initial_notgreen[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tng_aux >= 0)
                self.model.addConstr(tng_aux == tng[s] * self.colors[k, s, self.GREEN],
                                     'tng_aux_constr_{}_{}'.format(k, s))

                tng[s] += (1 - self.colors[k, s, self.GREEN]) - tng_aux
                self.model.addConstr(tng[s] <= self.initial_notgreen[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tng[s] >= 0)

                tng_history[k - 1, s] = tng[s]

        return tng_history

    ##########################
    # Initial constraints
    ##########################
    def initial_light_constraints(self, init):
        for s in range(self.configuration.num_signals):
            for c in range(self.num_colors):
                self.model.addConstr(self.colors[0, s, c] == init.lights[s, c],
                                     'init_light_{}_{}'.format(self.color_name[c], s))
