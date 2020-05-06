class SumoConfiguration:
    def __init__(self, config_file, tls_id, num_signals, lane_mapping_vec):
        self.config_file = config_file
        self.tls_id = tls_id
        self.num_signals = num_signals
        self.lane_mapping_vec = lane_mapping_vec
