import itertools

import gurobipy as gp
from gurobipy import GRB

import numpy as np


class GurobiIntersection:
    num_colors = 4

    GREEN = 0
    RED = 1
    YELLOW = 2
    AMBER = 3

    color_to_idx = {
        'green': GREEN,
        'red': RED,
        'yellow': YELLOW,
        'amber': AMBER
    }

    allcolors = [
        'green',
        'red',
        'yellow',
        'amber'
    ]

    def __init__(self, configuration, options):
        # Create a new model
        self.model = gp.Model('siren')
        self.model.Params.MIPGap = 0.03
        self.configuration = configuration
        self.options = options

        # Progress state
        self.initial_green = np.zeros(self.configuration.num_signals)
        self.initial_yellow = np.zeros(self.configuration.num_signals)
        self.initial_amber = np.zeros(self.configuration.num_signals)
        self.initial_notgreen = np.zeros(self.configuration.num_signals)
        self.initial_wait = np.zeros(self.configuration.num_signals)

        self.initial_lights = np.zeros((self.configuration.num_signals, self.num_colors))
        for s in range(self.configuration.num_signals):
            self.initial_lights[s, self.RED] = 1

        # Create variables
        self.colors = self.color_var()

        self.turned_green = self.lane_step_var('turned_green', inclusive=True, ub=1)
        self.turned_amber = self.lane_step_var('turned_amber', inclusive=True, ub=1)
        self.turned_yellow = self.lane_step_var('turned_yellow', inclusive=True, ub=1)
        # self.notgreen_timer = self.lane_step_var('notgreen_timer')

        self.queue_notempty = self.lane_step_var('queue_notempty', inclusive=True, skip_first=True, vtype=GRB.BINARY)
        self.queue = self.lane_step_var('queue', inclusive=True)
        self.flow = self.lane_step_var('flow', inclusive=True, skip_first=True)
        self.arrival = self.lane_step_var('arrival', inclusive=True, skip_first=True)
        self.departure = self.lane_step_var('departure', inclusive=True, skip_first=True)
        self.stops = self.lane_step_var('stops', inclusive=True, skip_first=True)
        self.wait_time = self.lane_step_var('wait_time', inclusive=True)
        self.request = self.lane_step_var('request', inclusive=True, skip_first=True, vtype=GRB.BINARY)

        # Add system dynamics
        self.queue_dynamics()

        # Timer dynamics
        self.turned_green_dynamics()
        self.turned_amber_dynamics()
        self.turned_yellow_dynamics()
        # self.notgreen_timer_dynamics()
        self.wait_time_timer_dynamics()

        # Add constraints
        self.single_light_constraints()
        self.conflict_constraints()
        self.green_transition_constraints()
        self.red_transition_constraints()
        self.yellow_transition_constraints()
        self.amber_transition_constraints()
        self.control_horizon_constraints()
        self.maximum_wait()

        # Add timing constraints
        self.min_green_constraints()
        self.amber_time_constraints()
        self.yellow_time_constraints()
        # self.green_interval()

        # Set object function
        queue_objective = self.queue_objective()
        stops_objective = self.stops_objective()
        wait_objective = self.wait_time_objective()
        green_objective = self.green_objective()
        throughput_objective = self.throughput_objective()

        self.model.setObjective(queue_objective * self.options.queue_weight +
                                stops_objective * self.options.stops_weight +
                                wait_objective * self.options.wait_weight +
                                green_objective * self.options.green_weight +
                                throughput_objective * self.options.throughput_weight)

        self.initial_set = False

        self.arrival_constraints = None
        self.departure_constraints = None
        self.green_interval_constraints = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals, self.configuration.num_signals), dtype=object)
        self.initial_wait_constraints = None
        self.initial_queue_constraints = None
        self.initial_light_constraints = None
        self.initial_notgreen_constraints = None
        self.initial_green_constraints = None
        self.initial_yellow_constraints = None
        self.initial_amber_constraints = None

    def optimize(self, queue, arrival, departure, verbose=False):
        self.init(queue, arrival, departure)

        self.model.reset()
        self.model.optimize()

        if self.model.status != GRB.OPTIMAL:
            self.iis(queue, arrival, departure)

        if verbose:
            print('')
            print('Variables:')

            for v in self.model.getVars():
                print('{}: {}'.format(v.varName, v.x))

            print('Obj: {}'.format(self.model.objVal))

        self.initial_lights = self.get_colors()

        for s in range(self.configuration.num_signals):
            if self.initial_lights[s, self.GREEN] > 0.9:
                self.initial_green[s] += 1
            else:
                self.initial_green[s] = 0
        for s in range(self.configuration.num_signals):
            if self.initial_lights[s, self.AMBER] > 0.9:
                self.initial_amber[s] += 1
            else:
                self.initial_amber[s] = 0
        for s in range(self.configuration.num_signals):
            if self.initial_lights[s, self.YELLOW] > 0.9:
                self.initial_yellow[s] += 1
            else:
                self.initial_yellow[s] = 0

        for s in range(self.configuration.num_signals):
            if self.initial_lights[s, self.GREEN] < 0.1:
                self.initial_notgreen[s] += 1
            else:
                self.initial_notgreen[s] = 0

        # self.initial_notgreen = np.vectorize(self.select_value)(self.notgreen_timer.select(1, '*'))
        self.initial_wait = np.vectorize(self.select_value)(self.wait_time.select(1, '*'))

        return self.get_colors()

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

    def get_colors(self, k=1):
        return np.vectorize(self.select_value)(self.colors.select(k, '*', '*')).reshape(self.configuration.num_signals,
                                                                                        self.num_colors)

    def initial_color_constraints(self, color, initial, timing):
        return self.model.addConstrs(
            (self.colors.sum([i for i in range(1, int(timing[s] - initial[s] + 1))], s, color) == timing[s] - initial[s]
                for s in range(self.configuration.num_signals) if 0 < initial[s] < timing[s]),
            'initial_{}'.format(color))

    def init(self, queue, arrival, departure):
        if self.initial_set:
            self.model.remove(self.arrival_constraints)
            self.model.remove(self.departure_constraints)
            self.model.remove(self.initial_wait_constraints)
            self.model.remove(self.initial_queue_constraints)
            self.model.remove(self.initial_light_constraints)
            # self.model.remove(self.initial_notgreen_constraints)
            self.model.remove(self.initial_green_constraints)
            self.model.remove(self.initial_yellow_constraints)
            self.model.remove(self.initial_amber_constraints)
            self.model.remove(self.green_interval_constraints)

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.arrival_constraints = self.model.addConstrs(
            (self.arrival[k, s] == int(arrival[k - 1, s]) for s, k in compound_range), 'arrival')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.departure_constraints = self.model.addConstrs(
            (self.departure[k, s] == int(departure[k - 1, s]) for s, k in compound_range), 'departure')

        self.initial_wait_constraints = self.model.addConstrs(
            (self.wait_time[0, s] == self.initial_wait[s] for s in range(self.configuration.num_signals)),
            'initial_wait')
        self.initial_queue_constraints = self.model.addConstrs(
            (self.queue[0, s] == queue[s] for s in range(self.configuration.num_signals)), 'initial_queue')

        compound_range = itertools.product(range(self.configuration.num_signals), self.allcolors)
        self.initial_light_constraints = self.model.addConstrs(
            (self.colors[0, s, color] == self.initial_lights[s, self.color_to_idx[color]] for s, color in
             compound_range), 'initial_lights')

        self.initial_green_constraints = self.initial_color_constraints('green', self.initial_green, self.configuration.min_green)
        self.initial_yellow_constraints = self.initial_color_constraints('yellow', self.initial_yellow, self.configuration.yellow_time)
        self.initial_amber_constraints = self.initial_color_constraints('amber', self.initial_amber, self.configuration.amber_time)

        # self.initial_notgreen_constraints = self.model.addConstrs(
        #     (self.notgreen_timer[0, s] == self.initial_notgreen[s] for s in range(self.configuration.num_signals)),
        #     'initial_notgreen')

        self.green_interval2()
        self.initial_set = True

    #####################
    # Create variables
    #####################
    def lane_step_var(self, name, inclusive=False, skip_first=False, vtype=GRB.CONTINUOUS, **kwargs):
        compound_range = itertools.product(range(skip_first, self.options.prediction_horizon + inclusive),
                                           range(self.configuration.num_signals))
        return self.model.addVars(compound_range, name=name, lb=0, vtype=vtype, **kwargs)

    def color_var(self):
        compound_range = itertools.product(range(self.options.prediction_horizon + 1),
                                           range(self.configuration.num_signals), self.allcolors)
        return self.model.addVars(compound_range, name='color', vtype=GRB.BINARY)

    #####################
    # System dynamics
    #####################
    def queue_dynamics(self):
        queue_upper_bound = 10000

        # Flow
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs((self.flow[k, s] <= self.queue[k - 1, s] + self.arrival[k, s] for s, k in compound_range),
                              'flow1')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.flow[k, s] <= queue_upper_bound * self.colors[k, s, 'green'] for s, k in compound_range), 'flow2')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs((self.flow[k, s] <= self.departure[k, s] for s, k in compound_range), 'flow3')

        # Queue
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.queue[k, s] == self.queue[k - 1, s] + self.arrival[k, s] - self.flow[k, s] for s, k in
             compound_range), 'queue1')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs((self.queue[k, s] >= self.queue_notempty[k, s] for s, k in compound_range), 'queue2')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.queue[k, s] <= queue_upper_bound * self.queue_notempty[k, s] for s, k in compound_range), 'queue3')

    #####################
    # Objectives
    #####################
    def queue_objective(self):
        return self.queue.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*')

    def stops_objective(self):
        stops_upper_bound = 10000

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.stops[k, s] <= stops_upper_bound * (1 - self.colors[k, s, 'green']) for s, k in compound_range),
            'stops1')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs((self.stops[k, s] <= self.arrival[k, s] for s, k in compound_range), 'stops2')

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.stops[k, s] >= self.arrival[k, s] - stops_upper_bound * self.colors[k, s, 'green'] for s, k in
             compound_range), 'stops3')

        return self.stops.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*')

    def wait_time_objective(self):
        return self.wait_time.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*')

    def green_objective(self):
        return -self.colors.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*', 'green')

    def throughput_objective(self):
        return -self.flow.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*')

    #########################
    # General constraints
    #########################
    def single_light_constraints(self):
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(self.options.prediction_horizon + 1))
        self.model.addConstrs((self.colors.sum(k, s, '*') == 1 for s, k in compound_range), 'single')

    def nonblocking(self, k, s):
        return 1 - self.colors[k, s, 'red']

    def conflict_constraints(self):
        compound_range = itertools.product(range(self.options.prediction_horizon + 1),
                                           range(self.configuration.num_signals), range(self.configuration.num_signals))
        self.model.addConstrs(
            (self.configuration.conflict_matrix[s1, s2] * (self.nonblocking(k, s1) + self.nonblocking(k, s2)) <= 1
             for k, s1, s2 in compound_range), 'blocking')

    def stable_light_transition_constraint(self, stable, after_not, after_zero, after_positive, timing):
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.colors[k - 1, s, stable] + self.colors[k, s, after_not] <= 1 for s, k in compound_range),
            'stable1_{}'.format(stable))

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            ((self.colors[k - 1, s, stable] + self.colors[k, s, after_zero]) * timing[s] <= 1 + timing[s] for s, k in
             compound_range), 'stable2_{}'.format(stable))

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.colors[k - 1, s, stable] + self.colors[k, s, after_positive] * (1 - timing[s]) <= 1 for s, k in
             compound_range), 'stable3_{}'.format(stable))

    def green_transition_constraints(self):
        self.stable_light_transition_constraint('green', 'amber', 'red', 'yellow',
                                                self.configuration.yellow_time)

    def red_transition_constraints(self):
        self.stable_light_transition_constraint('red', 'yellow', 'green', 'amber',
                                                self.configuration.amber_time)

    def intermediate_transition_constraints(self, intermediate, after1, after2):
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.colors[k - 1, s, intermediate] + self.colors[k, s, after1] <= 1 for s, k in compound_range),
            'intermediate1_{}'.format(intermediate))

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(1, self.options.prediction_horizon + 1))
        self.model.addConstrs(
            (self.colors[k - 1, s, intermediate] + self.colors[k, s, after2] <= 1 for s, k in compound_range),
            'intermediate2_{}'.format(intermediate))

    def yellow_transition_constraints(self):
        self.intermediate_transition_constraints('yellow', 'green', 'amber')

    def amber_transition_constraints(self):
        self.intermediate_transition_constraints('amber', 'yellow', 'red')

    def control_horizon_constraints(self):
        compound_range = itertools.product(range(self.options.control_horizon + 1, self.options.prediction_horizon),
                                           range(self.configuration.num_signals), self.allcolors)
        self.model.addConstrs((self.colors[k, s, c] == self.colors[k + 1, s, c] for k, s, c in compound_range),
                              'control_horizon')

    def maximum_wait(self):
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(self.options.prediction_horizon))
        self.model.addConstrs(
            (self.wait_time[k, s] <= self.configuration.maximum_wait[s] - 1 for s, k in compound_range if
             self.configuration.maximum_wait[s] > 0), 'maximum_wait')

    #########################################
    # Timer dynamics
    #########################################
    def turned_dynamics(self, turned, color):
        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(self.options.prediction_horizon))
        self.model.addConstrs((turned[k + 1, s] - self.colors[k + 1, s, color] <= 0 for s, k in compound_range),
                              'turned_{}1'.format(color))

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(self.options.prediction_horizon))
        self.model.addConstrs((turned[k + 1, s] - (1 - self.colors[k, s, color]) <= 0 for s, k in compound_range),
                              'turned_{}2'.format(color))

        compound_range = itertools.product(range(self.configuration.num_signals),
                                           range(self.options.prediction_horizon))
        self.model.addConstrs(
            (-turned[k + 1, s] + (1 - self.colors[k, s, color]) + self.colors[k + 1, s, color] <= 1 for s, k in
             compound_range), 'turned_{}3'.format(color))

    def turned_green_dynamics(self):
        self.turned_dynamics(self.turned_green, 'green')

    def turned_amber_dynamics(self):
        self.turned_dynamics(self.turned_amber, 'amber')

    def turned_yellow_dynamics(self):
        self.turned_dynamics(self.turned_yellow, 'yellow')

    def increment_counter(self, timer, k, s, count, reset, upper_bound):
        counter = timer[k, s]
        prev_counter = timer[k - 1, s]
        self.model.addConstr(counter <= upper_bound * count)
        self.model.addConstr(counter >= 0)

        self.model.addConstr(counter <= prev_counter + 1)
        self.model.addConstr(counter >= (prev_counter + 1) - upper_bound * reset)

    def notgreen_timer_dynamics(self):
        def upper_bound(k, s):
            return self.notgreen_timer[0, s] + k

        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon):
                self.increment_counter(self.notgreen_timer, k, s, self.colors.sum(k, s, ['red', 'yellow', 'amber']),
                                       self.colors[k, s, 'green'],
                                       upper_bound(k, s))

    def wait_time_timer_dynamics(self):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.request[k, s] - (1 - self.colors[k, s, 'green']) <= 0,
                                     'wait1_{}[{}]'.format(s, k))
                self.model.addConstr(self.request[k, s] - self.queue_notempty[k, s] <= 0, 'wait2_{}[{}]'.format(s, k))
                self.model.addConstr(
                    -self.request[k, s] + (1 - self.colors[k, s, 'green']) + self.queue_notempty[k, s] <= 1,
                    'wait3_{}[{}]'.format(s, k))
                self.increment_counter(self.wait_time, k, s, self.request[k, s], 1 - self.request[k, s],
                                       self.wait_time[0, s] + k)

    #########################################
    # Timing constraints
    #########################################
    def min_green_constraints(self):
        # General case
        for s in range(self.configuration.num_signals):
            min_green = self.configuration.min_green[s]
            if min_green > 0:
                for k in range(self.options.prediction_horizon + 1 - min_green):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, k + min_green)], s, 'green') >= min_green *
                        self.turned_green[k, s], 'min_green_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            min_green = self.configuration.min_green[s]
            if min_green > 0:
                for k in range(self.options.prediction_horizon + 1 - min_green, self.options.prediction_horizon):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'green') == (
                                self.options.prediction_horizon + 1 - k) * self.turned_green[k, s],
                        'min_green_e_{}[{}]'.format(s, k))

    def amber_time_constraints(self):
        # General case
        for s in range(self.configuration.num_signals):
            amber_time = self.configuration.amber_time[s]
            if amber_time > 0:
                for k in range(self.options.prediction_horizon + 1 - amber_time):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, k + amber_time)], s, 'amber') >= amber_time *
                        self.turned_amber[k, s], 'amber1_{}[{}]'.format(s, k))
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, k + amber_time + 1)], s, 'amber') <= amber_time,
                        name='amber2_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            amber_time = self.configuration.amber_time[s]
            if amber_time > 0:
                for k in range(self.options.prediction_horizon + 1 - amber_time, self.options.prediction_horizon):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'amber') == (
                                self.options.prediction_horizon + 1 - k) * self.turned_amber[k, s],
                        'amber_e_{}[{}]'.format(s, k))

    def yellow_time_constraints(self):
        # General case
        for s in range(self.configuration.num_signals):
            yellow_time = self.configuration.yellow_time[s]
            if yellow_time > 0:
                for k in range(self.options.prediction_horizon + 1 - yellow_time):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, k + yellow_time)], s, 'yellow') >= yellow_time *
                        self.turned_yellow[k, s], 'yellow1_{}[{}]'.format(s, k))
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, k + yellow_time + 1)], s, 'yellow') <= yellow_time,
                        name='yellow2_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            yellow_time = self.configuration.yellow_time[s]
            if yellow_time > 0:
                for k in range(self.options.prediction_horizon + 1 - yellow_time, self.options.prediction_horizon):
                    self.model.addConstr(
                        self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'yellow') == (
                                self.options.prediction_horizon + 1 - k) * self.turned_yellow[k, s],
                        'yellow_e_{}[{}]'.format(s, k))

    def green_interval(self):
        for s1 in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                green_diff = (1 - self.colors[k, s1, 'green']) - (1 - self.colors[k + 1, s1, 'green'])

                for s2 in range(self.configuration.num_signals):
                    self.model.addConstr(
                        self.configuration.green_interval[s2, s1] * green_diff <= self.notgreen_timer[k, s2],
                        'green_interval_{}_{}[{}]'.format(s1, s2, k))

    def green_interval2(self):
        compound_range = itertools.product(range(self.options.prediction_horizon + 1), range(self.configuration.num_signals), range(self.configuration.num_signals))

        def constraint(k, s1, s2):
            green_interval = self.configuration.green_interval[s2, s1]
            red_min = max(k - green_interval - 1, 1)
            red_max = max(k - 1, 1)
            return self.colors.sum([i for i in range(red_min, red_max)], s2, ['red', 'yellow', 'amber']) + int(
                self.initial_notgreen[s2]) >= green_interval * self.turned_green[k, s1]

        self.green_interval_constraints = self.model.addConstrs((constraint(k, s1, s2) for k, s1, s2 in compound_range if self.configuration.green_interval[s2, s1] > 0), 'green_interval')