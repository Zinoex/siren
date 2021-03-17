import json
import numpy as np

from gurobi.structures import GurobiConfiguration
from sumo.structures import SumoConfiguration


class ConfigurationLoader:
    def __init__(self, name):
        self.config_file = 'sumo/intersections/{}.sumocfg'.format(name)
        self.json_description_file = 'sumo/intersections/{}.desc.json'.format(name)

        # Open the description.json file and save all values to the class
        with open(self.json_description_file) as file:
            desc_data = json.loads(file.read())

        # Convert JSON keys to class member names
        self.intersection_name = desc_data['intersection_name']
        self.departure_rate = np.array(desc_data['departure_rate'])
        self.num_signals = desc_data['num_signals']

        self.gurobi_config = GurobiConfiguration(
            num_signals=self.num_signals,
            conflict_matrix=np.array(desc_data['conflict_matrix']),
            green_interval=np.array(desc_data['green_interval_matrix']),
            yellow_time=np.array(desc_data['time_vectors']['yellow']),
            amber_time=np.array(desc_data['time_vectors']['amber']),
            min_green=np.array(desc_data['time_vectors']['min_green']),
            red_clearance=np.array(desc_data['time_vectors']['red_clearance']),
            maximum_wait=np.array(desc_data['maximum_wait'])
        )

        self.sumo_config = SumoConfiguration(
            config_file=self.config_file,
            tls_id=desc_data['traffic_junction_id'],
            num_signals=self.num_signals,
            lane_mapping=desc_data['SUMO_lane_mapping'],
            non_protection={int(key): value for key, value in desc_data['non_protection'].items()},
            crossings=desc_data.get('crossings', [])
        )
