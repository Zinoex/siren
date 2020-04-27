
class Intersection:
    def __init__(self, num_signals, conflict_matrix, yellow_time, amber_time):
        self.num_signals = num_signals
        self.conflict_matrix = conflict_matrix
        self.yellow_time = yellow_time
        self.amber_time = amber_time


class Init:
    def __init__(self, lights, timing, queue):
        self.lights = lights
        self.timing = timing
        self.queue = queue


class Timing:
    def __init__(self, green, red, yellow, amber, not_green):
        self.green = green
        self.red = red
        self.yellow = yellow
        self.amber = amber
        self.not_green = not_green

