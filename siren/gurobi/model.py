from itertools import product
from time import time

import gurobipy as gp
from gurobipy import GRB

import numpy as np


class GurobiIntersection:
    num_colors = 3

    GREEN = 0
    RED = 1
    AMBER = 2

    color_to_idx = {
        'green': GREEN,
        'red': RED,
        'amber': AMBER
    }

    allcolors = [
        'green',
        'red',
        'amber'
    ]

    def __init__(self, configuration, options):
        self.timer = 0

        # Create a new model
        self.model = gp.Model('siren')
        # self.model.Params.MIPGap = 0.01
        # self.model.Params.PreQLinearize = 2
        # self.model.Params.RLTCuts = 2
        self.configuration = configuration
        self.options = options
        self.num_signals = np.arange(self.configuration.num_signals)
        self.prediction_horizon = np.arange(1, self.options.prediction_horizon + 1)

        # Progress state
        self.initial_green = np.zeros(self.configuration.num_signals)
        self.initial_amber = np.zeros(self.configuration.num_signals)
        self.initial_red_min = np.zeros(self.configuration.num_signals)
        self.initial_red_interval = np.zeros(self.configuration.num_signals)
        self.initial_wait = np.zeros(self.configuration.num_signals)

        self.initial_lights = np.zeros((self.configuration.num_signals, self.num_colors))
        for s in range(self.configuration.num_signals):
            self.initial_lights[s, self.RED] = 1

        self.initial_set = False

        self.arrival_constraints = None
        self.arrival_constraints1 = None
        self.arrival_constraints2 = None
        self.arrival_constraints3 = None
        self.arrival_constraints4 = None
        self.departure_constraints = None
        self.green_interval_constraints = None
        self.initial_wait_constraints = None
        self.initial_queue_constraints = None
        self.initial_light_constraints = None
        self.initial_red_constraints = None
        self.initial_green_constraints = None
        # self.initial_amber_constraints = None
        self.amber_constraints = None

        # Create variables
        self.colors = self.color_var()

        self.queue_notempty = self.lane_step_var('queue_notempty', inclusive=True, skip_first=True, vtype=GRB.BINARY)
        self.queue = self.lane_step_var('queue', inclusive=True)
        self.actual_flow = self.lane_step_var('actual_flow', inclusive=True, skip_first=True)
        self.stops = self.lane_step_var('stops', inclusive=True, skip_first=True)
        self.wait_time = self.lane_step_var('wait_time', inclusive=True)
        self.request = self.lane_step_var('request', inclusive=True, skip_first=True, vtype=GRB.BINARY)

        self.initial_wait_constraints = self.model.addConstrs((self.wait_time[0, s] == 0 for s in self.num_signals), 'initial_wait')
        self.initial_queue_constraints = self.model.addConstrs((self.queue[0, s] == 0 for s in self.num_signals), 'initial_queue')

        compound_range = product(self.num_signals, self.allcolors)
        self.initial_light_constraints = self.model.addConstrs((self.colors[0, s, c] == 0 for s, c in compound_range), 'initial_lights')

        # Add system dynamics
        self.queue_dynamics()

        # Timer dynamics
        self.wait_time_timer_dynamics()

        # Add constraints
        self.single_light_constraints()
        self.conflict_constraints()
        self.green_transition_constraints()
        self.red_transition_constraints()
        self.amber_transition_constraints()
        self.control_horizon_constraints()
        self.maximum_wait()

        # Add timing constraints
        self.min_green_constraints()
        self.amber_time_constraints()
        self.min_red_constraints()
        self.green_interval()

        # Set object function
        queue_objective = self.queue_objective()
        stops_objective = self.stops_objective()
        wait_objective = self.request_objective()
        green_objective = self.green_objective()
        throughput_objective = self.throughput_objective()

        self.model.setObjective(queue_objective * self.options.queue_weight +
                                stops_objective * self.options.stops_weight +
                                wait_objective * self.options.request_weight +
                                green_objective * self.options.green_weight +
                                throughput_objective * self.options.throughput_weight)

    def optimize(self, queue, arrival, departure, verbose=False):
        self.timer += 1

        print('Initial green', self.initial_green)
        print('Initial red min', self.initial_red_min)
        print('Initial red interval', self.initial_red_interval)
        print('Initial amber', self.initial_amber)

        t1 = time()
        self.init(queue, arrival, departure)
        t2 = time()

        self.model.reset()

        t3 = time()
        self.model.optimize()
        t4 = time()

        print('Model generation: {}, optimization: {}'.format(t2 - t1, t4 - t3))

        if self.model.status != GRB.OPTIMAL:
            self.iis(queue, arrival, departure)

        if verbose:
            print('')
            print('Variables:')

            for v in self.model.getVars():
                print('{}: {}'.format(v.varName, v.x))

            print('Obj: {}'.format(self.model.objVal))

        self.initial_lights = self.get_colors()

        self.increment_initial_timers(self.initial_green, self.GREEN, self.configuration.min_green)
        self.increment_initial_timers(self.initial_amber, self.AMBER, self.configuration.amber_time)
        self.increment_initial_timers(self.initial_red_min, self.RED, self.configuration.green_interval.max(axis=0))
        self.increment_initial_timers(self.initial_red_interval, self.RED,
                                      self.configuration.yellow_time + self.configuration.red_clearance)

        self.initial_wait = np.vectorize(self.select_int)(self.wait_time.select(1, '*'))

        return self.get_colors_with_yellow()

    def increment_initial_timers(self, initial, color, cap):
        for s in self.num_signals:
            if self.initial_lights[s, color] == 1:
                initial[s] = initial[s] + 1  # min(initial[s] + 1, cap[s])
            else:
                initial[s] = 0

    def iis(self, queue, arrival, departure, file='model'):
        self.init(queue, arrival, departure)

        self.model.computeIIS()
        self.model.write('{}.ilp'.format(file))

    def tune(self):
        self.model.tune()

        if self.model.tuneResultCount > 0:
            # Load the best tuned parameters into the model
            self.model.getTuneResult(0)

    @staticmethod
    def select_value(var):
        return var.x

    @staticmethod
    def select_int(var):
        return round(var.x)

    def get_colors(self, k=1):
        return np.vectorize(self.select_int)(self.colors.select(k, '*', '*')).reshape(self.configuration.num_signals, self.num_colors)

    def get_colors_with_yellow(self, k=1):
        colors = self.get_colors(k)

        yellow = np.int32(np.vectorize(
            lambda s: colors[s, self.RED] and self.initial_red_min[s] <= self.configuration.yellow_time[s] < self.timer
        )(self.num_signals))

        colors[:, self.RED] -= yellow

        return np.concatenate((colors, np.expand_dims(yellow, axis=1)), axis=1)

    def initial_color_constraints(self, color, initial, timing):
        return self.model.addConstrs((self.colors.sum(range(1, int(timing[s] - initial[s] + 1)), s, color) == timing[s] - initial[s] for s in self.num_signals if 0 < initial[s] < timing[s]), 'initial_{}'.format(color))

    def init(self, queue, arrival, departure):
        if self.initial_set:
            self.model.remove(self.initial_green_constraints)
            # self.model.remove(self.initial_amber_constraints)
            self.model.remove(self.initial_red_constraints)
            self.model.remove(self.departure_constraints)

        for s, k in product(filter(lambda s: self.configuration.amber_time[s] > 0, self.num_signals), self.prediction_horizon):
            amber_time = self.configuration.amber_time[s]
            self.amber_constraints[s, k].rhs = min(max(0, amber_time - k + 1), self.initial_amber[s])

        for s, k in product(self.num_signals, self.prediction_horizon):
            rhs = int(arrival[k - 1, s])
            self.arrival_constraints1[s, k].rhs = rhs
            self.arrival_constraints2[s, k].rhs = rhs
            self.arrival_constraints3[s, k].rhs = rhs
            self.arrival_constraints4[s, k].rhs = rhs

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.departure_constraints = self.model.addConstrs((self.actual_flow[k, s] <= int(departure[k - 1, s]) * self.colors[k, s, 'green'] for s, k in compound_range), 'flow2')

        for s in self.num_signals:
            self.initial_wait_constraints[s].rhs = self.initial_wait[s]
            self.initial_queue_constraints[s].rhs = queue[s]

        for s, c in product(self.num_signals, self.allcolors):
            self.initial_light_constraints[s, c].rhs = self.initial_lights[s, self.color_to_idx[c]]

        self.initial_green_constraints = self.initial_color_constraints('green', self.initial_green, self.configuration.min_green)
        # self.initial_amber_constraints = self.initial_color_constraints('amber', self.initial_amber, self.configuration.amber_time)
        self.initial_red_constraints = self.initial_color_constraints('red', self.initial_red_min, self.configuration.yellow_time + self.configuration.red_clearance)

        for k, s1, s2 in product(self.prediction_horizon, self.num_signals, self.num_signals):
            if self.configuration.green_interval[s2, s1] > 0:
                self.green_interval_constraints[k, s1, s2].rhs = -max(self.initial_red_interval[s2] - k + 1, 0)

        self.initial_set = True

    #####################
    # Create variables
    #####################
    def lane_step_var(self, name, inclusive=False, skip_first=False, vtype=GRB.CONTINUOUS, **kwargs):
        compound_range = product(range(skip_first, self.options.prediction_horizon + inclusive), self.num_signals)
        return self.model.addVars(compound_range, name=name, lb=0, vtype=vtype, **kwargs)

    def color_var(self):
        compound_range = product(range(self.options.prediction_horizon + 1), self.num_signals, self.allcolors)
        return self.model.addVars(compound_range, name='color', vtype=GRB.BINARY)

    #####################
    # System dynamics
    #####################
    def queue_dynamics(self):
        queue_upper_bound = 100

        # Flow = min(queue + arrival, departure * green)
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.arrival_constraints1 = self.model.addConstrs((self.actual_flow[k, s] - self.queue[k - 1, s] <= 0 for s, k in compound_range), 'flow1')

        # Queue = prev_queue + arrival - flow
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.arrival_constraints2 = self.model.addConstrs((self.queue[k, s] + self.actual_flow[k, s] - self.queue[k - 1, s] == 0 for s, k in compound_range), 'queue1')

        # Queue not empty dynamics
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.queue[k, s] >= self.queue_notempty[k, s] for s, k in compound_range), 'queue2')

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.queue[k, s] <= queue_upper_bound * self.queue_notempty[k, s] for s, k in compound_range), 'queue3')

    #####################
    # Objectives
    #####################
    def queue_objective(self):
        compound_range = product(self.prediction_horizon, self.num_signals)
        discount = {(k, s): self.options.discount_factor ** k for k, s in compound_range}

        return self.queue.prod(discount)

    def stops_objective(self):
        stops_upper_bound = 10000

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.stops[k, s] <= stops_upper_bound * (1 - self.colors[k, s, 'green']) for s, k in compound_range), 'stops1')

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.arrival_constraints3 = self.model.addConstrs((self.stops[k, s] <= 0 for s, k in compound_range), 'stops2')

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.arrival_constraints4 = self.model.addConstrs((self.stops[k, s] + stops_upper_bound * self.colors[k, s, 'green'] >= 0 for s, k in compound_range), 'stops3')

        compound_range = product(self.prediction_horizon, self.num_signals)
        discount = {(k, s): self.options.discount_factor ** k for k, s in compound_range}

        return self.stops.prod(discount)

    def request_objective(self):
        compound_range = product(range(self.options.prediction_horizon + 1), self.num_signals)
        discount = {(k, s): self.options.discount_factor ** k for k, s in compound_range}

        return self.request.prod(discount)

    def green_objective(self):
        compound_range = product(range(self.options.prediction_horizon + 1), self.num_signals)
        discount = {(k, s, 'green'): self.options.discount_factor ** k for k, s in compound_range}

        return -self.colors.prod(discount, '*', '*', 'green')

    def throughput_objective(self):
        compound_range = product(self.prediction_horizon, self.num_signals)
        discount = {(k, s): self.options.discount_factor ** k for k, s in compound_range}

        return -self.actual_flow.prod(discount)

    #########################
    # General constraints
    #########################
    def single_light_constraints(self):
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.colors.sum(k, s, '*') == 1 for s, k in compound_range), 'single')

    def nonblocking(self, k, s):
        return 1 - self.colors[k, s, 'red']

    def conflict_constraints(self):
        compound_range = product(range(1, self.options.prediction_horizon + 1), self.num_signals, self.num_signals)
        self.model.addConstrs((self.nonblocking(k, s1) + self.nonblocking(k, s2) <= 1 for k, s1, s2 in compound_range if self.configuration.conflict_matrix[s1, s2] == 1), 'blocking')

    def green_transition_constraints(self):
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.colors[k - 1, s, 'green'] + self.colors[k, s, 'amber'] <= 1 for s, k in compound_range), 'green')

    def red_transition_constraints(self):
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.colors[k - 1, s, 'red'] + self.colors[k, s, 'amber' if self.configuration.amber_time[s] == 0 else 'green'] <= 1 for s, k in compound_range), 'red')

    def amber_transition_constraints(self):
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.colors[k - 1, s, 'amber'] + self.colors[k, s, 'red'] <= 1 for s, k in compound_range), 'intermediate_amber')

    def control_horizon_constraints(self):
        compound_range = product(range(self.options.control_horizon + 1, self.options.prediction_horizon), self.num_signals, self.allcolors)
        self.model.addConstrs((self.colors[k, s, c] == self.colors[k + 1, s, c] for k, s, c in compound_range), 'control_horizon')

    def maximum_wait(self):
        compound_range = product(self.num_signals, range(self.options.prediction_horizon))
        self.model.addConstrs((self.wait_time[k, s] <= self.configuration.maximum_wait[s] - 1 for s, k in compound_range if self.configuration.maximum_wait[s] > 0), 'maximum_wait')

    #########################################
    # Timer dynamics
    #########################################
    def wait_time_timer_dynamics(self):
        # if (not green) and queue_not_empty then request
        compound_range = product(self.num_signals, self.prediction_horizon)
        not_green = lambda k, s: 1 - self.colors[k, s, 'green']

        self.model.addConstrs((not_green(k, s) + self.queue_notempty[k, s] <= 1 + self.request[k, s] for s, k in compound_range), 'wait1')
        self.model.addConstrs((self.queue_notempty[k, s] >= self.request[k, s] for s, k in compound_range), 'wait2')
        self.model.addConstrs((not_green(k, s) >= self.request[k, s] for s, k in compound_range), 'wait3')


        # self.wait_time[s, k] = if self.request[s, k] then self.wait_time[s, k - 1] + 1 else 0
        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.wait_time[k, s] <= self.configuration.maximum_wait[s] * self.request[k, s] for s, k in compound_range), 'wait_timer1')

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.wait_time[k, s] <= self.wait_time[k - 1, s] + 1 for s, k in compound_range), 'wait_timer2')

        compound_range = product(self.num_signals, self.prediction_horizon)
        self.model.addConstrs((self.wait_time[k, s] >= (self.wait_time[k - 1, s] + 1) - self.configuration.maximum_wait[s] * (1 - self.request[k, s]) for s, k in compound_range), 'wait_timer3')

    #########################################
    # Timing constraints
    #########################################
    def min_green_constraints(self):
        # General case
        def generator():
            for s in self.num_signals:
                for k in range(1, self.options.prediction_horizon + 1 - self.configuration.min_green[s]):
                    yield s, k

        def constraint(s, k):
            min_green = self.configuration.min_green[s]
            green_diff = self.colors[k, s, 'green'] - self.colors[k - 1, s, 'green']
            return self.colors.sum(range(k, k + min_green), s, 'green') >= min_green * green_diff

        self.model.addConstrs((constraint(s, k) for s, k in generator()), 'min_green')

        # End range case
        def generator():
            for s in self.num_signals:
                for k in range(self.options.prediction_horizon + 1 - self.configuration.min_green[s], self.options.prediction_horizon):
                    yield s, k

        def constraint(s, k):
            green_diff = self.colors[k, s, 'green'] - self.colors[k - 1, s, 'green']

            return self.colors.sum(range(k, self.options.prediction_horizon + 1), s, 'green') >= \
                   (self.options.prediction_horizon + 1 - k) * green_diff

        self.model.addConstrs((constraint(s, k) for s, k in generator()), 'min_green_e')

    def min_red_constraints(self):
        # General case
        def generator():
            for s in self.num_signals:
                min_red = self.configuration.yellow_time[s] + self.configuration.red_clearance[s]
                for k in range(1, self.options.prediction_horizon + 1 - min_red):
                    yield s, k

        def constraint(s, k):
            min_red = self.configuration.yellow_time[s] + 1
            red_diff = self.colors[k, s, 'red'] - self.colors[k - 1, s, 'red']
            return self.colors.sum(range(k, k + min_red), s, 'red') >= min_red * red_diff

        self.model.addConstrs((constraint(s, k) for s, k in generator()), 'min_red')

        # End range case
        def generator():
            for s in self.num_signals:
                min_red = self.configuration.yellow_time[s] + self.configuration.red_clearance[s]
                for k in range(self.options.prediction_horizon + 1 - min_red, self.options.prediction_horizon):
                    yield s, k

        def constraint(s, k):
            red_diff = self.colors[k, s, 'red'] - self.colors[k - 1, s, 'red']

            return self.colors.sum(range(k, self.options.prediction_horizon + 1), s, 'red') >= \
                   (self.options.prediction_horizon + 1 - k) * red_diff

        self.model.addConstrs((constraint(s, k) for s, k in generator()), 'min_red_e')

    def amber_time_constraints(self):
        # # General case
        # def generator():
        #     for s in self.num_signals:
        #         amber_time = self.configuration.amber_time[s]
        #         if amber_time > 0:
        #             for k in range(1, self.options.prediction_horizon + 1 - amber_time):
        #                 yield s, k
        #
        # def constraint(s, k):
        #     amber_time = self.configuration.amber_time[s]
        #     amber_diff = self.colors[k, s, 'amber'] - self.colors[k - 1, s, 'amber']
        #     return self.colors.sum(range(k, k + amber_time), s, 'amber') >= amber_time * amber_diff
        #
        # self.model.addConstrs((constraint(s, k) for s, k in generator()), 'amber1')
        #
        # def constraint(s, k):
        #     amber_time = self.configuration.amber_time[s]
        #     return self.colors.sum(range(k, k + amber_time + 1), s, 'amber') <= amber_time
        #
        # self.model.addConstrs((constraint(s, k) for s, k in generator()), 'amber2')
        #
        # # End range case
        # def generator():
        #     for s in self.num_signals:
        #         amber_time = self.configuration.amber_time[s]
        #         if amber_time > 0:
        #             for k in range(self.options.prediction_horizon + 1 - amber_time, self.options.prediction_horizon):
        #                 yield s, k
        #
        # def constraint(s, k):
        #     amber_diff = self.colors[k, s, 'amber'] - self.colors[k - 1, s, 'amber']
        #     return self.colors.sum(range(k, self.options.prediction_horizon + 1), s, 'amber') >= (self.options.prediction_horizon + 1 - k) * amber_diff
        #
        # self.model.addConstrs((constraint(s, k) for s, k in generator()), 'amber_e')

        # General case

        def constraint(s, k):
            amber_time = self.configuration.amber_time[s]
            amber_diff = self.colors[k, s, 'amber'] - self.colors[k - 1, s, 'amber']

            lf_ra = min(amber_time - 1, self.configuration.prediction_horizon - k) + 1
            return self.colors.sum(range(k, k + lf_ra), s, 'amber') >= amber_diff * lf_ra

        compound_range = product(filter(lambda s: self.configuration.amber_time[s] > 0, self.num_signals), self.prediction_horizon)
        self.model.addConstrs((constraint(s, k) for s, k in compound_range), 'amber1')

        def constraint(s, k):
            amber_time = self.configuration.amber_time[s]
            lb_ra = min(k, amber_time) - 1

            return self.colors[k, s, 'amber'] * amber_time - self.colors.sum(range(k - lb_ra, k + 1), s, 'amber') >= 0

        compound_range = product(filter(lambda s: self.configuration.amber_time[s] > 0, self.num_signals), self.prediction_horizon)
        self.amber_constraints = self.model.addConstrs((constraint(s, k) for s, k in compound_range), 'amber2')

    def green_interval(self):
        compound_range = product(self.prediction_horizon, self.num_signals, self.num_signals)

        def constraint(k, s1, s2):
            # (initial_red + current_red) - green interval * green_diff >= 0
            green_interval = self.configuration.green_interval[s2, s1]
            current_red_start = max(k - green_interval, 1)

            current_red = self.colors.sum(range(current_red_start, k), s2, 'red')
            green_diff = self.colors[k, s1, 'green'] - self.colors[k - 1, s1, 'green']

            return current_red - green_interval * green_diff >= 0

        self.green_interval_constraints = self.model.addConstrs((constraint(k, s1, s2) for k, s1, s2 in compound_range if self.configuration.green_interval[s2, s1] > 0), 'green_interval')