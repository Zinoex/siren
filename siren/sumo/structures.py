class SumoConfiguration:
    def __init__(self, config_file, tls_id, num_signals, lane_mapping, crossings=None):
        self.config_file = config_file
        self.tls_id = tls_id
        self.num_signals = num_signals
        self.lane_mapping = lane_mapping
        self.crossings = crossings
