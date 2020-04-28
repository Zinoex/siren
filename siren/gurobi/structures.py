
class Configuration:
    def __init__(self, num_signals, conflict_matrix, green_interval, yellow_time, amber_time, min_green):
        self.num_signals = num_signals
        self.conflict_matrix = conflict_matrix
        self.green_interval = green_interval
        self.yellow_time = yellow_time
        self.amber_time = amber_time
        self.min_green = min_green


class Initialization:
    def __init__(self, lights, timing, queue):
        self.lights = lights
        self.timing = timing
        self.queue = queue


class Timing:
    def __init__(self, green, yellow, amber, not_green, wait):
        self.green = green
        self.yellow = yellow
        self.amber = amber
        self.not_green = not_green
        self.wait = wait


class Options:
    def __init__(self, prediction_horizon=20, queue_weight=1, stops_weight=1, wait_weight=3):
        self.prediction_horizon = prediction_horizon
        self.queue_weight = queue_weight
        self.stops_weight = stops_weight
        self.wait_weight = wait_weight
