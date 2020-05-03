class Configuration:
    def __init__(self, num_signals, conflict_matrix, green_interval, yellow_time, amber_time, min_green):
        self.num_signals = num_signals
        self.conflict_matrix = conflict_matrix
        self.green_interval = green_interval
        self.yellow_time = yellow_time
        self.amber_time = amber_time
        self.min_green = min_green


class Options:
    def __init__(self, prediction_horizon=30, control_horizon=30,
                 queue_weight=1, stops_weight=1,
                 wait_weight=1, green_weight=0.01):
        self.prediction_horizon = prediction_horizon
        self.control_horizon = control_horizon
        self.queue_weight = queue_weight
        self.stops_weight = stops_weight
        self.wait_weight = wait_weight
        self.green_weight = green_weight
