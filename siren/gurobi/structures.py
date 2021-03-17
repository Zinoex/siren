class GurobiConfiguration:
    def __init__(self, num_signals, conflict_matrix, green_interval, yellow_time, amber_time, min_green, red_clearance, maximum_wait):
        self.num_signals = num_signals
        self.conflict_matrix = conflict_matrix
        self.green_interval = green_interval
        self.yellow_time = yellow_time
        self.amber_time = amber_time
        self.min_green = min_green
        self.red_clearance = red_clearance
        self.maximum_wait = maximum_wait


class GurobiOptions:
    def __init__(self, prediction_horizon=30, control_horizon=20,
                 queue_weight=1, stops_weight=6,
                 request_weight=0.05, green_weight=0.01, throughput_weight=2,
                 discount_factor=0.97, **kwargs):
        self.prediction_horizon = prediction_horizon
        self.control_horizon = control_horizon
        self.queue_weight = queue_weight
        self.stops_weight = stops_weight
        self.request_weight = request_weight
        self.green_weight = green_weight
        self.throughput_weight = throughput_weight
        self.discount_factor = discount_factor


class ConstantDeparture:
    def __init__(self, rate=2):
        self.rate = rate

    def __getitem__(self, item):
        return self.rate


class ConstantArrival:
    def __init__(self, rate=1):
        self.rate = rate

    def __getitem__(self, item):
        return self.rate


class PerLaneDeparture:
    def __init__(self, rates):
        self.rates = rates

    def __getitem__(self, item):
        _, s = item
        return self.rates[s]
