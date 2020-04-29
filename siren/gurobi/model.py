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
        self.min_green_constraints()

        self.initial_amber = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_amber[s] = gp.LinExpr(init.timing.amber[s])
        self.amber_time_constraints()

        self.initial_yellow = np.empty(self.configuration.num_signals, dtype=object)
        for s in range(self.configuration.num_signals):
            self.initial_yellow[s] = gp.LinExpr(init.timing.yellow[s])
        self.yellow_time_constraints()

        # self.initial_notgreen = np.empty(self.configuration.num_signals, dtype=object)
        # for s in range(self.configuration.num_signals):
        #     self.initial_notgreen[s] = gp.LinExpr(init.timing.not_green[s])
        # self.green_interval()

        self.initial_light_constraints(init)

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
        # self.model.Params.TimeLimit = 2

        # Try to set a feasible solution for optimized performance
        self.initial_solution(init)

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

    def initial_solution(self, init):
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon + 1):
                self.colors[k, s, self.GREEN] = init.lights[s, self.GREEN]
                self.colors[k, s, self.RED] = init.lights[s, self.RED]
                self.colors[k, s, self.YELLOW] = init.lights[s, self.YELLOW]
                self.colors[k, s, self.AMBER] = init.lights[s, self.AMBER]

                if init.lights[s, self.YELLOW] == 1 and self.initial_yellow[s] + k >= self.configuration.yellow_time[s]:
                    self.colors[k, s, self.YELLOW] = 0
                    self.colors[k, s, self.RED] = 1

                if init.lights[s, self.AMBER] == 1 and self.initial_amber[s] + k >= self.configuration.amber_time[s]:
                    self.colors[k, s, self.AMBER] = 0
                    self.colors[k, s, self.GREEN] = 1

    #####################
    # System dynamics
    #####################
    def queue_evolution(self, arr_fn, dep_fn):
        objective = gp.LinExpr()

        queue_notempty = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for s in range(self.configuration.num_signals):
            queue = self.initial_queue[s]
            for k in range(1, self.options.prediction_horizon + 1):
                q_res = queue + arr_fn(k, s) - dep_fn(k, s) * (
                            self.colors[k, s, self.GREEN] + self.colors[k, s, self.YELLOW])

                q = self.model.addVar(vtype=GRB.BINARY, name='q_{}_{}'.format(k, s))
                self.model.addGenConstrIndicator(q, False, q_res <= 0, name='q_residual_{}_{}'.format(k, s))
                q_prime = self.model.addVar(vtype=GRB.INTEGER, name='q_prime_{}_{}'.format(k, s))
                self.model.addConstr(q_prime == q * q_res)

                queue = q_prime
                queue_notempty[k - 1, s] = q

                # Update objective function
                objective.add(queue)

        return objective, queue_notempty

    def stops_evolution(self, arr_fn):
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                objective.add(arr_fn(k, s) * (self.colors[k, s, self.RED] + self.colors[k, s, self.AMBER]))

        return objective

    def wait_time_evolution(self, queue_notempty):
        objective = gp.LinExpr()

        for s in range(self.configuration.num_signals):
            tq = self.initial_wait_time[s]
            for k in range(1, self.options.prediction_horizon + 1):
                q_notempty = queue_notempty[k - 1, s]

                tq_aux = self.model.addVar(vtype=GRB.INTEGER, name='tq_aux_{}_{}'.format(k, s))
                self.model.addConstr(tq_aux == tq * (1 - q_notempty), 'tq_aux_constr_{}_{}'.format(k, s))

                tq += q_notempty - tq_aux

                # Update objective function
                objective.add(tq)

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
        tg_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for s in range(self.configuration.num_signals):
            tg = self.initial_green[s]
            for k in range(1, self.options.prediction_horizon + 1):
                # Constrain min green
                self.model.addConstr(
                    self.configuration.min_green[s] * (self.colors[k - 1, s, self.GREEN] - self.colors[k, s, self.GREEN]) <= tg,
                    'min_green_{}_{}'.format(k, s))

                # Update tg
                tg_aux = self.model.addVar(vtype=GRB.INTEGER, name='tg_aux_{}_{}'.format(k, s))
                self.model.addConstr(tg_aux <= self.initial_green[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tg_aux >= 0)
                self.model.addConstr(tg_aux == (tg + 1) * self.colors[k, s, self.GREEN],
                                     'tg_aux_constr_{}_{}'.format(k, s))

                tg = tg_aux

    def amber_time_constraints(self):
        ma = max(self.configuration.amber_time)

        ta_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for s in range(self.configuration.num_signals):
            ta = self.initial_amber[s]
            for k in range(1, self.options.prediction_horizon + 1):
                # Constrain amber time
                ta_product = self.logical_product((1 - self.colors[k, s, self.AMBER]), self.colors[k - 1, s, self.AMBER], name='ta_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.amber_time[s] - ta) * ta_product == 0,
                                     'amber_time_max_{}_{}'.format(k, s))

                # Update ta
                ta_aux = self.model.addVar(vtype=GRB.INTEGER, name='ta_aux_{}_{}'.format(k, s))
                self.model.addConstr(ta_aux <= ma)
                self.model.addConstr(ta_aux >= 0)
                self.model.addConstr(ta_aux == (ta + 1) * self.colors[k, s, self.AMBER], 'ta_aux_constr_{}_{}'.format(k, s))

                ta = ta_aux

                self.model.addConstr(self.configuration.amber_time[s] * self.colors[k, s, self.AMBER] >= ta,
                                     'amber_time_min_{}_{}'.format(k, s))

    def yellow_time_constraints(self):
        my = max(self.configuration.yellow_time)

        ty_history = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

        for s in range(self.configuration.num_signals):
            ty = self.initial_yellow[s]
            for k in range(1, self.options.prediction_horizon + 1):
                # Constrain amber time
                ty_product = self.logical_product((1 - self.colors[k, s, self.YELLOW]), self.colors[k - 1, s, self.YELLOW], name='ty_product_{}_{}'.format(k, s))

                self.model.addConstr((self.configuration.yellow_time[s] - ty) * ty_product == 0,
                                     'yellow_time_max_{}_{}'.format(k, s))

                # Update ta
                ty_aux = self.model.addVar(vtype=GRB.INTEGER, name='ty_aux_{}_{}'.format(k, s))
                self.model.addConstr(ty_aux <= my)
                self.model.addConstr(ty_aux >= 0)
                self.model.addConstr(ty_aux == (ty + 1) * self.colors[k, s, self.YELLOW], 'ty_aux_constr_{}_{}'.format(k, s))

                ty = ty_aux

                self.model.addConstr(self.configuration.yellow_time[s] * self.colors[k, s, self.YELLOW] >= ty,
                                     'yellow_time_min_{}_{}'.format(k, s))

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
                tng_product = self.logical_product(self.colors[k, s1, self.GREEN], 1 - self.colors[k - 1, s1, self.GREEN], name='tng_product_{}_{}'.format(k, s1))

                # for s2 in range(self.configuration.num_signals):
                #     # Constrain green interval
                #     v = self.model.addVar(vtype=GRB.INTEGER, name='time_missing_{}_{}_{}'.format(k, s1, s2))
                #     self.model.addConstr(v == (self.configuration.green_interval[s2, s1] - tng[s2]) * tng_product)
                #     self.model.addConstr(
                #         (self.configuration.green_interval[s2, s1] - tng[s2]) * tng_product <= 0,
                #         'green_interval_{}_{}_{}'.format(k, s1, s2))

            for s in range(self.configuration.num_signals):
                # Update tng
                tng_aux = self.model.addVar(vtype=GRB.INTEGER, name='tng_aux_{}_{}'.format(k, s))
                self.model.addConstr(tng_aux <= self.initial_notgreen[s] + self.options.prediction_horizon + 1 - k)
                self.model.addConstr(tng_aux >= 0)
                self.model.addConstr(tng_aux == (tng[s] + 1) * (1 - self.colors[k, s, self.GREEN]),
                                     'tng_aux_constr_{}_{}'.format(k, s))

                tng[s] = tng_aux

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
