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

        # Create variables

        # Colors
        self.colors = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals, self.num_colors),
                               dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.colors[k, s, c] = self.model.addVar(vtype=GRB.BINARY,
                                                             name='{}_{}_{}'.format(self.color_name[c], k, s))

        # Color negations
        self.notcolors = np.empty(
            (self.options.prediction_horizon + 1, self.configuration.num_signals, self.num_colors),
            dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.notcolors[k, s, c] = self.model.addVar(vtype=GRB.BINARY,
                                                             name='not_{}_{}_{}'.format(self.color_name[c], k, s))

        # Timers
        self.green_timer = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                self.green_timer[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='green_timer_{}_{}'.format(k, s))

        self.amber_timer = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.amber_timer[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='amber_timer_{}_{}'.format(k, s))

        self.yellow_timer = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.yellow_timer[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='yellow_timer_{}_{}'.format(k, s))

        self.notgreen_timer = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                self.notgreen_timer[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='notgreen_timer_{}_{}'.format(k, s))

        # Queue variables
        self.queue_notempty = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.queue_notempty[k, s] = self.model.addVar(vtype=GRB.BINARY, name='queue_notempty_{}_{}'.format(k, s))

        self.queue = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.queue[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='queue_{}_{}'.format(k, s))

        self.queue_prime = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.queue_prime[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='queue_prime_{}_{}'.format(k, s))

        self.wait_time = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.wait_time[k, s] = self.model.addVar(vtype=GRB.CONTINUOUS, name='wait_time_{}_{}'.format(k, s))

        self.request = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.request[k, s] = self.model.addVar(vtype=GRB.BINARY, name='request_{}_{}'.format(k, s))

        # Add system dynamics
        self.queue_dynamics(arr_fn, dep_fn)

        # Timer dynamics
        self.green_timer_dynamics(init)
        self.amber_timer_dynamics(init)
        self.yellow_timer_dynamics(init)
        self.notgreen_timer_dynamics(init)
        self.wait_time_timer_dynamics(init)

        # Add constraints
        self.light_duality_constraints()
        self.single_light_constraints()
        self.conflict_constraints()
        self.green_transition_constraints()
        self.red_transition_constraints()
        self.yellow_transition_constraints()
        self.amber_transition_constraints()
        self.control_horizon_constraints()

        # Add timing constraints
        self.min_green_constraints()
        self.amber_time_constraints()
        self.yellow_time_constraints()
        self.green_interval()

        self.initial_light_constraints(init)
        self.initial_queue(init)

        # Set object function
        queue_objective = self.queue_objective()
        stops_objective = self.stops_objective(arr_fn)
        wait_objective = self.wait_time_objective()
        green_objective = self.green_objective()

        self.model.setObjectiveN(queue_objective, 0, weight=self.options.queue_weight)
        self.model.setObjectiveN(stops_objective, 1, weight=self.options.stops_weight)
        self.model.setObjectiveN(wait_objective, 2, weight=self.options.wait_weight)
        self.model.setObjectiveN(green_objective, 3, weight=self.options.green_weight)

    def optimize(self, verbose=False):
        self.model.optimize()

        if verbose:
            print('')
            print('Variables:')

            for v in self.model.getVars():
                print('[{}]: {}'.format(v.varName, v.x))

            print('Obj: {}'.format(self.model.objVal))

    def iis(self, file='model'):
        self.model.computeIIS()
        self.model.write('{}.iis'.format(file))

    def tune(self):
        self.model.tune()

        if self.model.tuneResultCount > 0:
            # Load the best tuned parameters into the model
            self.model.getTuneResult(0)

    def get_colors(self, k=1):
        def select_value(color):
            return color.x

        return np.vectorize(select_value)(self.colors[k, :, :])

    #####################
    # System dynamics
    #####################
    def queue_dynamics(self, arr_fn, dep_fn):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.queue_prime[k, s] == self.queue[k - 1, s] + arr_fn(k, s) - dep_fn(k, s) * self.colors[k, s, self.GREEN])
                self.model.addGenConstrIndicator(self.queue_notempty[k, s], False, self.queue_prime[k, s] <= 0)
                self.model.addConstr(self.queue[k, s] == self.queue_notempty[k, s] * self.queue_prime[k, s])

    #####################
    # Objectives
    #####################
    def queue_objective(self):
        return self.queue.sum()

    def stops_objective(self, arr_fn):
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                objective.add(arr_fn(k, s) * self.notcolors[k, s, self.GREEN])

        return objective

    def wait_time_objective(self):
        return self.wait_time.sum()

    def green_objective(self):
        return self.notcolors[:, :, self.GREEN].sum()

    #########################
    # General constraints
    #########################
    def light_duality_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.model.addConstr(self.notcolors[k, s, c] == 1 - self.colors[k, s, c])

    def single_light_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(self.colors[k, s, :].sum() == 1)

    def nonblocking(self, k, s):
        return self.notcolors[k, s, self.RED]

    def conflict_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s1 in range(self.configuration.num_signals):
                for s2 in range(self.configuration.num_signals):
                    self.model.addConstr(self.configuration.conflict_matrix[s1, s2] * (
                            self.nonblocking(k, s1) + self.nonblocking(k, s2)) <= 1)

    def stable_light_transition_constraint(self, stable, after_not, after_zero, after_positive, timing):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, stable] + self.colors[k, s, after_not] <= 1)
                self.model.addConstr(
                    (self.colors[k - 1, s, stable] + self.colors[k, s, after_zero]) * timing[s] <= 1 + timing[s])
                self.model.addConstr(
                    self.colors[k - 1, s, stable] + self.colors[k, s, after_positive] * (1 - timing[s]) <= 1)

    def green_transition_constraints(self):
        self.stable_light_transition_constraint(self.GREEN, self.AMBER, self.RED, self.YELLOW,
                                                self.configuration.yellow_time)

    def red_transition_constraints(self):
        self.stable_light_transition_constraint(self.RED, self.YELLOW, self.GREEN, self.AMBER,
                                                self.configuration.amber_time)

    def intermediate_transition_constraints(self, intermediate, after1, after2):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after1] <= 1)
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after2] <= 1)

    def yellow_transition_constraints(self):
        self.intermediate_transition_constraints(self.YELLOW, self.GREEN, self.AMBER)

    def amber_transition_constraints(self):
        self.intermediate_transition_constraints(self.AMBER, self.YELLOW, self.RED)

    def control_horizon_constraints(self):
        for k in range(self.options.control_horizon + 1, self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    self.model.addConstr(self.colors[k, s, c] == self.colors[k + 1, s, c])

    #########################################
    # Timer dynamics
    #########################################
    def increment_counter(self, timer, k, s, count, reset, upper_bound):
        counter = timer[k, s]
        prev_counter = timer[k - 1, s]
        self.model.addConstr(counter <= upper_bound * count)
        self.model.addConstr(counter >= 0)

        self.model.addConstr(counter == (prev_counter + 1) * count)
        self.model.addConstr(counter <= prev_counter + 1)
        self.model.addConstr(counter >= (prev_counter + 1) - upper_bound * reset)

        return counter

    def timer_dynamics(self, timer, initial_timing, upper_bound, c, reverse=False, inclusive=False):
        count = self.colors if not reverse else self.notcolors
        reset = self.notcolors if not reverse else self.colors

        for s in range(self.configuration.num_signals):
            self.model.addConstr(timer[0, s] == initial_timing[s])
            for k in range(1, self.options.prediction_horizon + inclusive):
                self.increment_counter(timer, k, s, count[k, s, c], reset[k, s, c],
                                       upper_bound(k, s) if callable(upper_bound) else upper_bound)

    def green_timer_dynamics(self, init):
        def upper_bound(k, s):
            return init.timing.green[s] + k

        self.timer_dynamics(self.green_timer, init.timing.green, upper_bound, self.GREEN)

    def amber_timer_dynamics(self, init):
        ma = max(self.configuration.amber_time)

        self.timer_dynamics(self.amber_timer, init.timing.amber, ma + 1, self.AMBER, inclusive=True)

    def yellow_timer_dynamics(self, init):
        my = max(self.configuration.yellow_time)

        self.timer_dynamics(self.yellow_timer, init.timing.yellow, my + 1, self.YELLOW, inclusive=True)

    def notgreen_timer_dynamics(self, init):
        def upper_bound(k, s):
            return init.timing.not_green[s] + k

        self.timer_dynamics(self.notgreen_timer, init.timing.not_green, upper_bound, self.GREEN, reverse=True)

    def wait_time_timer_dynamics(self, init):
        for s in range(self.configuration.num_signals):
            self.model.addConstr(self.wait_time[0, s] == init.timing.wait[s])
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addGenConstrAnd(self.request[k, s], [self.notcolors[k, s, self.GREEN], self.queue_notempty[k, s]])
                self.increment_counter(self.wait_time, k, s, self.request[k, s], 1 - self.request[k, s],
                                       init.timing.wait[s] + self.options.prediction_horizon)

    #########################################
    # Timing constraints
    #########################################
    def min_green_constraints(self):
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                green_diff = self.colors[k, s, self.GREEN] - self.colors[k + 1, s, self.GREEN]
                self.model.addConstr(
                    self.configuration.min_green[s] * green_diff <= self.green_timer[k, s])

    def amber_time_constraints(self):
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                amber_diff = self.colors[k, s, self.AMBER] - self.colors[k + 1, s, self.AMBER]

                self.model.addConstr(self.configuration.amber_time[s] * amber_diff <= self.amber_timer[k, s])
                self.model.addConstr(self.configuration.amber_time[s] * self.colors[k + 1, s, self.AMBER] >= self.amber_timer[k + 1, s])

    def yellow_time_constraints(self):
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                yellow_diff = self.colors[k, s, self.YELLOW] - self.colors[k + 1, s, self.YELLOW]

                self.model.addConstr(self.configuration.yellow_time[s] * yellow_diff <= self.yellow_timer[k, s])
                self.model.addConstr(self.configuration.yellow_time[s] * self.colors[k + 1, s, self.YELLOW] >= self.yellow_timer[k + 1, s])

    def green_interval(self):
        for s1 in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                green_diff = self.notcolors[k, s1, self.GREEN] - self.notcolors[k + 1, s1, self.GREEN]

                for s2 in range(self.configuration.num_signals):
                    self.model.addConstr(self.configuration.green_interval[s2, s1] * green_diff <= self.notgreen_timer[k, s2])

    ##########################
    # Initial constraints
    ##########################
    def initial_light_constraints(self, init):
        for s in range(self.configuration.num_signals):
            for c in range(self.num_colors):
                self.model.addConstr(self.colors[0, s, c] == init.lights[s, c])

    def initial_queue(self, init):
        for s in range(self.configuration.num_signals):
            self.model.addConstr(self.queue[0, s] == init.queue[s])
