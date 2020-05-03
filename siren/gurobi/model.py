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

    def __init__(self, configuration, options):
        # Create a new model
        self.model = gp.Model('siren')
        self.configuration = configuration
        self.options = options

        # Create variables
        self.colors = self.color_var('')
        self.notcolors = self.color_var('not_')

        self.green_timer = self.lane_step_var('green_timer')
        self.amber_timer = self.lane_step_var('amber_timer', inclusive=True)
        self.yellow_timer = self.lane_step_var('yellow_timer', inclusive=True)
        self.notgreen_timer = self.lane_step_var('notgreen_timer')

        self.queue_notempty = self.lane_step_var('queue_notempty', inclusive=True, skip_first=True, vtype=GRB.BINARY)
        self.queue = self.lane_step_var('queue', inclusive=True)
        self.queue_prime = self.lane_step_var('queue_prime', inclusive=True, skip_first=True)
        self.arrival = self.lane_step_var('arrival', inclusive=True, skip_first=True)
        self.departure = self.lane_step_var('departure', inclusive=True, skip_first=True)
        self.stops = self.lane_step_var('stops', inclusive=True, skip_first=True)
        self.wait_time = self.lane_step_var('wait_time', inclusive=True)
        self.request = self.lane_step_var('request', inclusive=True, skip_first=True, vtype=GRB.BINARY)

        # Add system dynamics
        self.queue_dynamics()

        # Timer dynamics
        self.green_timer_dynamics()
        self.amber_timer_dynamics()
        self.yellow_timer_dynamics()
        self.notgreen_timer_dynamics()
        self.wait_time_timer_dynamics()

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

        # Set object function
        queue_objective = self.queue_objective()
        stops_objective = self.stops_objective()
        wait_objective = self.wait_time_objective()
        green_objective = self.green_objective()

        self.model.setObjectiveN(queue_objective, 0, weight=self.options.queue_weight)
        self.model.setObjectiveN(stops_objective, 1, weight=self.options.stops_weight)
        self.model.setObjectiveN(wait_objective, 2, weight=self.options.wait_weight)
        self.model.setObjectiveN(green_objective, 3, weight=self.options.green_weight)

        self.initial_set = False
        self.initial_constraints = np.empty((6 + self.num_colors, self.configuration.num_signals), dtype=object)
        self.arrival_constraints = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)
        self.departure_constraints = np.empty((self.options.prediction_horizon, self.configuration.num_signals), dtype=object)

    def optimize(self, init, arrival, departure, verbose=False):
        self.init(init, arrival, departure)

        self.model.optimize()

        if verbose:
            print('')
            print('Variables:')

            for v in self.model.getVars():
                print('[{}]: {}'.format(v.varName, v.x))

            print('Obj: {}'.format(self.model.objVal))

    def iis(self, init, arrival, departure, file='model'):
        self.init(init, arrival, departure)

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

    def init(self, init, arrival, departure):
        if self.initial_set:
            self.model.remove(self.initial_constraints.flatten().tolist())
            self.model.remove(self.arrival_constraints.flatten().tolist())
            self.model.remove(self.departure_constraints.flatten().tolist())

        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.arrival_constraints[k - 1, s] = self.model.addConstr(self.arrival[k, s] == arrival[k, s])
                self.departure_constraints[k - 1, s] = self.model.addConstr(self.departure[k, s] == departure[k, s])

        for s in range(self.configuration.num_signals):
            self.initial_constraints[0, s] = self.model.addConstr(self.green_timer[0, s] == init.timing.green[s])
            self.initial_constraints[1, s] = self.model.addConstr(self.amber_timer[0, s] == init.timing.amber[s])
            self.initial_constraints[2, s] = self.model.addConstr(self.yellow_timer[0, s] == init.timing.yellow[s])
            self.initial_constraints[3, s] = self.model.addConstr(self.notgreen_timer[0, s] == init.timing.not_green[s])
            self.initial_constraints[4, s] = self.model.addConstr(self.wait_time[0, s] == init.timing.wait[s])

            self.initial_constraints[5, s] = self.model.addConstr(self.queue[0, s] == init.queue[s])

            for c in range(self.num_colors):
                self.initial_constraints[6 + c, s] = self.model.addConstr(self.colors[0, s, c] == init.lights[s, c])

        self.initial_set = True

    #####################
    # Create variables
    #####################
    def lane_step_var(self, name, inclusive=False, skip_first=False, vtype=GRB.CONTINUOUS):
        var = np.empty((self.options.prediction_horizon + inclusive, self.configuration.num_signals), dtype=object)
        for k in range(skip_first, self.options.prediction_horizon + inclusive):
            for s in range(self.configuration.num_signals):
                var[k, s] = self.model.addVar(vtype=vtype, name='{}_{}_{}'.format(name, k, s))

        return var

    def color_var(self, prefix):
        colors = np.empty((self.options.prediction_horizon + 1, self.configuration.num_signals, self.num_colors), dtype=object)
        for k in range(self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                for c in range(self.num_colors):
                    colors[k, s, c] = self.model.addVar(vtype=GRB.BINARY, name='{}{}_{}_{}'.format(prefix, self.color_name[c], k, s))

        return colors

    #####################
    # System dynamics
    #####################
    def queue_dynamics(self):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addConstr(self.queue_prime[k, s] == self.queue[k - 1, s] + self.arrival[k, s] - self.departure[k, s] * self.colors[k, s, self.GREEN])
                self.model.addGenConstrIndicator(self.queue_notempty[k, s], False, self.queue_prime[k, s] <= 0)

                self.model.addConstr(self.queue[k, s] <= 1000 * self.queue_notempty[k, s])
                self.model.addConstr(self.queue[k, s] >= 0)

                self.model.addConstr(self.queue[k, s] <= self.queue_prime[k, s])
                self.model.addConstr(self.queue[k, s] >= self.queue_prime[k, s] - 1000 * (1 - self.queue_notempty[k, s]))

    #####################
    # Objectives
    #####################
    def queue_objective(self):
        return self.queue[1:, :].sum()

    def stops_objective(self):
        objective = gp.LinExpr()

        for k in range(1, self.options.prediction_horizon + 1):
            for s in range(self.configuration.num_signals):
                self.model.addConstr(self.stops[k, s] <= 1000 * self.notcolors[k, s, self.GREEN])
                self.model.addConstr(self.stops[k, s] >= 0)

                self.model.addConstr(self.stops[k, s] <= self.arrival[k, s])
                self.model.addConstr(self.stops[k, s] >= self.arrival[k, s] - 1000 * self.colors[k, s, self.GREEN])

                objective.add(self.stops[k, s])

        return objective

    def wait_time_objective(self):
        return self.wait_time[1:, :].sum()

    def green_objective(self):
        return self.notcolors[1:, :, self.GREEN].sum()

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

        self.model.addConstr(counter <= prev_counter + 1)
        self.model.addConstr(counter >= (prev_counter + 1) - upper_bound * reset)

        return counter

    def timer_dynamics(self, timer, upper_bound, c, reverse=False, inclusive=False):
        count = self.colors if not reverse else self.notcolors
        reset = self.notcolors if not reverse else self.colors

        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + inclusive):
                self.increment_counter(timer, k, s, count[k, s, c], reset[k, s, c],
                                       upper_bound(k, s) if callable(upper_bound) else upper_bound)

    def green_timer_dynamics(self):
        def upper_bound(k, s):
            return self.green_timer[0, s] + k

        self.timer_dynamics(self.green_timer, upper_bound, self.GREEN)

    def amber_timer_dynamics(self):
        ma = max(self.configuration.amber_time)

        self.timer_dynamics(self.amber_timer, ma + 1, self.AMBER, inclusive=True)

    def yellow_timer_dynamics(self):
        my = max(self.configuration.yellow_time)

        self.timer_dynamics(self.yellow_timer, my + 1, self.YELLOW, inclusive=True)

    def notgreen_timer_dynamics(self):
        def upper_bound(k, s):
            return self.notgreen_timer[0, s] + k

        self.timer_dynamics(self.notgreen_timer, upper_bound, self.GREEN, reverse=True)

    def wait_time_timer_dynamics(self):
        for s in range(self.configuration.num_signals):
            for k in range(1, self.options.prediction_horizon + 1):
                self.model.addGenConstrAnd(self.request[k, s], [self.notcolors[k, s, self.GREEN], self.queue_notempty[k, s]])
                self.increment_counter(self.wait_time, k, s, self.request[k, s], 1 - self.request[k, s],
                                       self.wait_time[0, s] + k)

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
