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

    color_name = {
        GREEN: 'green',
        RED: 'red',
        YELLOW: 'yellow',
        AMBER: 'amber'
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
        self.notgreen_timer = self.lane_step_var('notgreen_timer')

        self.queue_notempty = self.lane_step_var('queue_notempty', inclusive=True, skip_first=True, ub=1)
        self.queue = self.lane_step_var('queue', inclusive=True)
        self.flow = self.lane_step_var('flow', inclusive=True, skip_first=True)
        self.arrival = self.lane_step_var('arrival', inclusive=True, skip_first=True)
        self.departure = self.lane_step_var('departure', inclusive=True, skip_first=True)
        self.stops = self.lane_step_var('stops', inclusive=True, skip_first=True)
        self.wait_time = self.lane_step_var('wait_time', inclusive=True)
        self.request = self.lane_step_var('request', inclusive=True, skip_first=True, ub=1)

        # Add system dynamics
        self.queue_dynamics()

        # Timer dynamics
        self.green_timer_dynamics()
        self.turned_amber_dynamics()
        self.turned_yellow_dynamics()
        self.notgreen_timer_dynamics()
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
        self.green_interval()

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
        self.initial_constraints = np.empty((6 + self.num_colors, self.configuration.num_signals), dtype=object)
        self.arrival_constraints = np.empty((self.options.prediction_horizon, self.configuration.num_signals),
                                            dtype=object)
        self.departure_constraints = np.empty((self.options.prediction_horizon, self.configuration.num_signals),
                                              dtype=object)
        self.green_interval_constraints = np.empty((self.options.prediction_horizon, self.configuration.num_signals, self.configuration.num_signals),
                                              dtype=object)

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
                print('[{}]: {}'.format(v.varName, v.x))

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

        # for s in range(self.configuration.num_signals):
        #     if self.initial_lights[s, 'green'] < 0.1:
        #         self.initial_notgreen[s] += 1
        #     else:
        #         self.initial_notgreen[s] = 0

        self.initial_notgreen = np.vectorize(self.select_value)(self.notgreen_timer[1, :])
        self.initial_wait = np.vectorize(self.select_value)(self.wait_time[1, :])

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
        return np.vectorize(self.select_value)(self.colors.select(k, '*', '*')).reshape(self.configuration.num_signals, self.num_colors)

    def init(self, queue, arrival, departure):
        if self.initial_set:
            self.model.remove(self.initial_constraints.flatten().tolist())
            self.model.remove(self.arrival_constraints.flatten().tolist())
            self.model.remove(self.departure_constraints.flatten().tolist())
            # self.model.remove(self.green_interval_constraints.flatten().tolist())

        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.arrival_constraints[k - 1, s] = self.model.addConstr(self.arrival[k, s] == int(arrival[k - 1, s]), 'arrival_{}[{}]'.format(s, k))
                self.departure_constraints[k - 1, s] = self.model.addConstr(self.departure[k, s] == int(departure[k - 1, s]), 'departure_{}[{}]'.format(s, k))

        for s in range(self.configuration.num_signals):
            self.initial_constraints[0, s] = self.model.addConstr(self.wait_time[0, s] == self.initial_wait[s], 'initial_wait_{}'.format(s))
            self.initial_constraints[1, s] = self.model.addConstr(self.queue[0, s] == queue[s], 'initial_queue_{}'.format(s))

            for idx, color in enumerate(self.allcolors):
                self.initial_constraints[2 + idx, s] = self.model.addConstr(
                    self.colors[0, s, color] == self.initial_lights[s, idx], 'initial_lights_{}[{}]'.format(s, color))

        # Start range case
        for s in range(self.configuration.num_signals):
            min_green = self.configuration.min_green[s]

            if 0 < self.initial_green[s] < min_green:
                self.initial_constraints[6, s] = self.model.addConstr(self.colors.sum([i for i in range(1, int(min_green - self.initial_green[s] + 1))], s, 'green') == min_green - self.initial_green[s], 'initial_green_{}'.format(s))
            else:
                self.initial_constraints[6, s] = self.model.addConstr(0 == 0, 'initial_green_trivial_{}'.format(s))

        # Start range case
        for s in range(self.configuration.num_signals):
            amber_time = self.configuration.amber_time[s]

            if 0 < self.initial_amber[s] < amber_time:
                self.initial_constraints[7, s] = self.model.addConstr(self.colors.sum([i for i in range(1, int(amber_time - self.initial_amber[s] + 1))], s, 'amber') == amber_time - self.initial_amber[s], 'initial_amber_{}'.format(s))
            else:
                self.initial_constraints[7, s] = self.model.addConstr(0 == 0, 'initial_amber_trivial_{}'.format(s))

        # Start range case
        for s in range(self.configuration.num_signals):
            yellow_time = self.configuration.yellow_time[s]

            if 0 < self.initial_yellow[s] < yellow_time:
                self.initial_constraints[8, s] = self.model.addConstr(self.colors.sum([i for i in range(1, int(yellow_time - self.initial_yellow[s] + 1))], s, 'yellow') == int(yellow_time - self.initial_yellow[s]), 'initial_yellow_{}'.format(s))
            else:
                self.initial_constraints[8, s] = self.model.addConstr(0 == 0, 'initial_yellow_trivial_{}'.format(s))

        for s in range(self.configuration.num_signals):
            self.initial_constraints[9, s] = self.model.addConstr(self.notgreen_timer[0, s] == self.initial_notgreen[s],
                                                              'initial_notgreen_{}'.format(s))
        #
        # self.green_interval()
        self.initial_set = True

    #####################
    # Create variables
    #####################
    def lane_step_var(self, name, inclusive=False, skip_first=False, vtype=GRB.CONTINUOUS, **kwargs):
        var = np.empty((self.options.prediction_horizon + inclusive, self.configuration.num_signals), dtype=object)
        for k in range(skip_first, self.options.prediction_horizon + inclusive):
            for s in range(self.configuration.num_signals):
                var[k, s] = self.model.addVar(vtype=vtype, name='{}_{}_{}'.format(name, k, s), lb=0, **kwargs)

        return var

    def color_var(self):
        generator = itertools.product(range(self.options.prediction_horizon + 1), range(self.configuration.num_signals), self.allcolors)

        return self.model.addVars(generator, name='color', vtype=GRB.BINARY)

    #####################
    # System dynamics
    #####################
    def queue_dynamics(self):
        queue_upper_bound = 100
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                # Flow
                self.model.addConstr(self.flow[k, s] <= self.queue[k - 1, s] + self.arrival[k, s], 'flow1_{}[{}]'.format(s, k))
                self.model.addConstr(self.flow[k, s] <= queue_upper_bound * self.colors[k, s, 'green'], 'flow2_{}[{}]'.format(s, k))
                self.model.addConstr(self.flow[k, s] <= self.departure[k, s], 'flow3_{}[{}]'.format(s, k))

                # Queue
                self.model.addConstr(self.queue[k, s] == self.queue[k - 1, s] + self.arrival[k, s] - self.flow[k, s], 'queue1_{}[{}]'.format(s, k))
                self.model.addConstr(self.queue[k, s] >= self.queue_notempty[k, s], 'queue2_{}[{}]'.format(s, k))
                self.model.addConstr(self.queue[k, s] <= queue_upper_bound * self.queue_notempty[k, s], 'queue3_{}[{}]'.format(s, k))

    #####################
    # Objectives
    #####################
    def queue_objective(self):
        return self.queue[1:, :].sum()

    def stops_objective(self):
        stops_upper_bound = 100
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(self.stops[k, s] <= stops_upper_bound * (1 - self.colors[k, s, 'green']), 'stops1_{}[{}]'.format(s, k))
                self.model.addConstr(self.stops[k, s] >= 0, 'stops2_{}[{}]'.format(s, k))

                self.model.addConstr(self.stops[k, s] <= self.arrival[k, s], 'stops3_{}[{}]'.format(s, k))
                self.model.addConstr(self.stops[k, s] >= self.arrival[k, s] - stops_upper_bound * self.colors[k, s, 'green'], 'stops4_{}[{}]'.format(s, k))

                objective.add(self.stops[k, s])

        return objective

    def wait_time_objective(self):
        return self.wait_time[1:, :].sum()

    def green_objective(self):
        return -self.colors.sum([i for i in range(1, self.options.prediction_horizon + 1)], '*', 'green')

    def throughput_objective(self):
        return -self.flow[1:, :].sum()

    #########################
    # General constraints
    #########################
    def single_light_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(self.colors.sum(k, s, '*') == 1, 'single_{}[{}]'.format(s, k))

    def nonblocking(self, k, s):
        return 1 - self.colors[k, s, 'red']

    def conflict_constraints(self):
        for k in range(self.options.prediction_horizon + 1):
            for s1 in range(self.configuration.num_signals):
                for s2 in range(self.configuration.num_signals):
                    self.model.addConstr(self.configuration.conflict_matrix[s1, s2] * (
                            self.nonblocking(k, s1) + self.nonblocking(k, s2)) <= 1, 'blocking_{}_{}[{}]'.format(s1, s2, k))

    def stable_light_transition_constraint(self, stable, after_not, after_zero, after_positive, timing):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, stable] + self.colors[k, s, after_not] <= 1, 'stable1_{}_{}[{}]'.format(stable, s, k))
                self.model.addConstr(
                    (self.colors[k - 1, s, stable] + self.colors[k, s, after_zero]) * timing[s] <= 1 + timing[s], 'stable2_{}_{}[{}]'.format(stable, s, k))
                self.model.addConstr(
                    self.colors[k - 1, s, stable] + self.colors[k, s, after_positive] * (1 - timing[s]) <= 1, 'stable3_{}_{}[{}]'.format(stable, s, k))

    def green_transition_constraints(self):
        self.stable_light_transition_constraint('green', 'amber', 'red', 'yellow',
                                                self.configuration.yellow_time)

    def red_transition_constraints(self):
        self.stable_light_transition_constraint('red', 'yellow', 'green', 'amber',
                                                self.configuration.amber_time)

    def intermediate_transition_constraints(self, intermediate, after1, after2):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after1] <= 1, 'intermediate1_{}_{}[{}]'.format(intermediate, s, k))
                self.model.addConstr(self.colors[k - 1, s, intermediate] + self.colors[k, s, after2] <= 1, 'intermediate2_{}_{}[{}]'.format(intermediate, s, k))

    def yellow_transition_constraints(self):
        self.intermediate_transition_constraints('yellow', 'green', 'amber')

    def amber_transition_constraints(self):
        self.intermediate_transition_constraints('amber', 'yellow', 'red')

    def control_horizon_constraints(self):
        for k in range(self.options.control_horizon + 1, self.options.prediction_horizon):
            for s in range(self.configuration.num_signals):
                for c in self.allcolors:
                    self.model.addConstr(self.colors[k, s, c] == self.colors[k + 1, s, c], 'control_horizon_{}_{}[{}]'.format(c, s, k))

    def maximum_wait(self):
        for s in range(self.configuration.num_signals):
            if self.configuration.maximum_wait[s] > 0:
                for k in range(0, self.options.prediction_horizon):
                    self.model.addConstr(self.wait_time[k, s] <= self.configuration.maximum_wait[s] - 1, 'maximum_wait_{}[{}]'.format(s, k))

    #########################################
    # Timer dynamics
    #########################################
    def increment_counter(self, timer, k, s, count, reset, upper_bound):
        counter = timer[k, s]
        prev_counter = timer[k - 1, s]
        self.model.addConstr(counter <= upper_bound * count)
        self.model.addConstr(counter >= 0)

        self.model.addConstr(counter <= prev_counter + 1)
        self.model.addConstr(counter >= (prev_counter + 1) - upper_bound * reset)

    def green_timer_dynamics(self):
        # Turned green
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                self.model.addConstr(self.turned_green[k + 1, s] - self.colors[k + 1, s, 'green'] <= 0, 'turned_green1_{}[{}]'.format(s, k))
                self.model.addConstr(self.turned_green[k + 1, s] - (1 - self.colors[k, s, 'green']) <= 0, 'turned_green2_{}[{}]'.format(s, k))
                self.model.addConstr(-self.turned_green[k + 1, s] + (1 - self.colors[k, s, 'green']) + self.colors[k + 1, s, 'green'] <= 1, 'turned_green3_{}[{}]'.format(s, k))

    def turned_amber_dynamics(self):
        # Turned amber
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                self.model.addConstr(self.turned_amber[k + 1, s] - self.colors[k + 1, s, 'amber'] <= 0, 'turned_amber1_{}[{}]'.format(s, k))
                self.model.addConstr(self.turned_amber[k + 1, s] - (1 - self.colors[k, s, 'amber']) <= 0, 'turned_amber2_{}[{}]'.format(s, k))
                self.model.addConstr(-self.turned_amber[k + 1, s] + (1 - self.colors[k, s, 'amber']) + self.colors[k + 1, s, 'amber'] <= 1, 'turned_amber3_{}[{}]'.format(s, k))

    def turned_yellow_dynamics(self):
        # Turned yellow
        for s in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                self.model.addConstr(self.turned_yellow[k + 1, s] - self.colors[k + 1, s, 'yellow'] <= 0, 'turned_yellow1_{}[{}]'.format(s, k))
                self.model.addConstr(self.turned_yellow[k + 1, s] - (1 - self.colors[k, s, 'yellow']) <= 0, 'turned_yellow2_{}[{}]'.format(s, k))
                self.model.addConstr(-self.turned_yellow[k + 1, s] + (1 - self.colors[k, s, 'yellow']) + self.colors[k + 1, s, 'yellow'] <= 1, 'turned_yellow3_{}[{}]'.format(s, k))

    def notgreen_timer_dynamics(self):
        def upper_bound(k, s):
            return self.notgreen_timer[0, s] + k

        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon):
                self.increment_counter(self.notgreen_timer, k, s, self.colors.sum(k, s, ['red', 'yellow', 'amber']), self.colors[k, s, 'green'],
                                       upper_bound(k, s))

    def wait_time_timer_dynamics(self):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.request[k, s] - (1 - self.colors[k, s, 'green']) <= 0, 'wait1_{}[{}]'.format(s, k))
                self.model.addConstr(self.request[k, s] - self.queue_notempty[k, s] <= 0, 'wait2_{}[{}]'.format(s, k))
                self.model.addConstr(-self.request[k, s] + (1 - self.colors[k, s, 'green']) + self.queue_notempty[k, s] <= 1, 'wait3_{}[{}]'.format(s, k))
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
                    self.model.addConstr(self.colors.sum([i for i in range(k, k + min_green)], s, 'green') >= min_green * self.turned_green[k, s], 'min_green_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            min_green = self.configuration.min_green[s]
            if min_green > 0:
                for k in range(self.options.prediction_horizon + 1 - min_green, self.options.prediction_horizon):
                    self.model.addConstr(self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'green') == (self.options.prediction_horizon + 1 - k) * self.turned_green[k, s], 'min_green_e_{}[{}]'.format(s, k))

    def amber_time_constraints(self):
        # General case
        for s in range(self.configuration.num_signals):
            amber_time = self.configuration.amber_time[s]
            if amber_time > 0:
                for k in range(self.options.prediction_horizon + 1 - amber_time):
                    self.model.addConstr(self.colors.sum([i for i in range(k, k + amber_time)], s, 'amber') >= amber_time * self.turned_amber[k, s], 'amber1_{}[{}]'.format(s, k))
                    self.model.addConstr(self.colors.sum([i for i in range(k, k + amber_time + 1)], s, 'amber') <= amber_time, name='amber2_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            amber_time = self.configuration.amber_time[s]
            if amber_time > 0:
                for k in range(self.options.prediction_horizon + 1 - amber_time, self.options.prediction_horizon):
                    self.model.addConstr(self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'amber') == (self.options.prediction_horizon + 1 - k) * self.turned_amber[k, s], 'amber_e_{}[{}]'.format(s, k))

    def yellow_time_constraints(self):
        # General case
        for s in range(self.configuration.num_signals):
            yellow_time = self.configuration.yellow_time[s]
            if yellow_time > 0:
                for k in range(self.options.prediction_horizon + 1 - yellow_time):
                    self.model.addConstr(self.colors.sum([i for i in range(k, k + yellow_time)], s, 'yellow') >= yellow_time * self.turned_yellow[k, s], 'yellow1_{}[{}]'.format(s, k))
                    self.model.addConstr(self.colors.sum([i for i in range(k, k + yellow_time + 1)], s, 'yellow') <= yellow_time, name='yellow2_{}[{}]'.format(s, k))

        # End range case
        for s in range(self.configuration.num_signals):
            yellow_time = self.configuration.yellow_time[s]
            if yellow_time > 0:
                for k in range(self.options.prediction_horizon + 1 - yellow_time, self.options.prediction_horizon):
                    self.model.addConstr(self.colors.sum([i for i in range(k, self.options.prediction_horizon + 1)], s, 'yellow') == (self.options.prediction_horizon + 1 - k) * self.turned_yellow[k, s], 'yellow_e_{}[{}]'.format(s, k))

    def green_interval(self):
        for s1 in range(self.configuration.num_signals):
            for k in range(self.options.prediction_horizon):
                green_diff = (1 - self.colors[k, s1, 'green']) - (1 - self.colors[k + 1, s1, 'green'])

                for s2 in range(self.configuration.num_signals):
                    # green_interval = self.configuration.green_interval[s2, s1]
                    # red_min = max(k - green_interval, 1)
                    # red_max = max(k, 1)
                    # 'green'_interval_constraints[k - 1, s1, s2] = self.model.addConstr(
                    #     (1 - self.colors)[red_min:red_max, s2, 'green'].sum() + int(self.initial_notgreen[s2]) == green_interval * self.turned_green[k, s1],
                    #     'green_interval_{}_{}[{}]'.format(s1, s2, k))
                    self.model.addConstr(
                        self.configuration.green_interval[s2, s1] * green_diff <= self.notgreen_timer[k, s2], 'green_interval_{}_{}[{}]'.format(s1, s2, k))
