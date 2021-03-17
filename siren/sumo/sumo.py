import os
import sys
from sumolib import checkBinary
import traci
import numpy as np


class SUMOSimulation:
    COLOR_TO_INDEX = {'r': 1, 'y': 2, 'g': 0, 'G': 0, 's': 2, 'u': 3}
    INDEX_TO_COLOR = ['g', 'r', 'u', 'y']

    def __init__(self, configuration, options):
        self.simulation_step = 0
        self.time_step_len = 1.0  # Temporary (seconds per timestep)

        self.prediction_horizon = options.prediction_horizon
        self.max_iterations = options.iterations
        self.configuration = configuration
        self.options = options

        self.lane_queue = None
        self.lane_map = {}
        self.num_tls_lanes = 0

        self.test_sumo_home()

        self.sumo_cmd = [self.sumo_binary(options),
                         '-c', self.configuration.config_file,
                         '--tripinfo-output', 'data/{}/tripinfo.xml'.format(options.folder_prefix),
                         '--summary', 'data/{}/summary.xml'.format(options.folder_prefix),
                         '--device.emissions.probability', '1',
                         '--seed', '42',
                         '-Q',
                         '--step-length', '0.05']

    @staticmethod
    def test_sumo_home():
        if 'SUMO_HOME' in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            sys.exit('SUMO_HOME not declared in path.')

    @staticmethod
    def sumo_binary(options):
        if options.nogui:
            binary = checkBinary('sumo')
            print('Command line version detected.')
        else:
            binary = checkBinary('sumo-gui')
            print('GUI detected.')

        return binary

    def map_lanes_to_signals(self):
        # Create lane_map dictionary to turn lanes to signals
        signal_lanes = traci.trafficlight.getControlledLanes(self.configuration.tls_id)

        self.lane_map = {lane: self.configuration.lane_mapping[i] for i, lane in enumerate(signal_lanes)}

    def start(self, clear=True):
        print('Starting Sumo Simulation.')
        traci.start(self.sumo_cmd)

        self.map_lanes_to_signals()

        # Constant number of lanes is used a lot
        self.num_tls_lanes = len(traci.trafficlight.getControlledLanes(self.configuration.tls_id))

        if clear:
            # Set lights to off (default state)
            traci.trafficlight.setRedYellowGreenState(self.configuration.tls_id, 'o' * self.num_tls_lanes)

    @property
    def lights(self):
        light_mat = np.zeros((self.configuration.num_signals, 4))

        # Get the current light state
        lane_light_state = traci.trafficlight.getRedYellowGreenState(self.configuration.tls_id)

        # Loop through each lane and update the signal matrix by the correct value
        for lane_idx, light in enumerate(lane_light_state):
            light_mat[self.configuration.lane_mapping[lane_idx], self.COLOR_TO_INDEX[light]] = 1

        return light_mat

    @lights.setter
    def lights(self, light_matrix):
        # Set default as off (could be red too) to allow indexing of the list
        output_light_list = ['o'] * self.num_tls_lanes

        # Argmax gives which light is a 1 for each signal
        # Assumes light rows are one-hot-encoded
        for i, color_idx in enumerate(np.argmax(light_matrix, axis=1)):
            for j, sig_idx in enumerate(self.configuration.lane_mapping):
                if sig_idx == i:
                    output_light_list[j] = self.INDEX_TO_COLOR[color_idx]

        for i in range(len(self.configuration.lane_mapping)):
            non_protection = self.configuration.non_protection.get(i, [])
            if output_light_list[i] == 'g' and all([output_light_list[j] in ['r', 'y'] for j in non_protection]):
                output_light_list[i] = 'G'

        traci.trafficlight.setRedYellowGreenState(self.configuration.tls_id, ''.join(output_light_list))

    @property
    def queue(self):
        # self.lane_queue was described using tls lanes, we want signals
        signal_queue = np.zeros(self.configuration.num_signals)
        self.update_lane_queue()

        for i, lane_queue in enumerate(self.lane_queue):
            signal_queue[self.configuration.lane_mapping[i]] += lane_queue

        return signal_queue

    def update_lane_queue(self):
        self.lane_queue = np.zeros(self.num_tls_lanes)

        self.update_vehicle_queue()
        self.update_pedestrian_queue()

    def update_vehicle_queue(self, stop_speed_thresh=3., stop_dist_thresh=100.):
        # Get list of all lane ids connected to the traffic light
        tls_lane_ids = traci.trafficlight.getControlledLanes(self.configuration.tls_id)

        # This is to not double count vehicles (might be a little hacky)
        counted_vehicles = []

        for tls_idx, tls_lane in enumerate(tls_lane_ids):
            for vehicle in np.unique(traci.lane.getLastStepVehicleIDs(tls_lane)):
                speed = traci.vehicle.getSpeed(vehicle)
                # getNextTLS returns a list of tuples (tls_id, tls_index, distance, state)
                # We only care about the distance of the first traffic light
                _, _, remaining_dist, _ = traci.vehicle.getNextTLS(vehicle)[0]

                # If it's going slow enough && it's close enough && we haven't counted it
                if speed < stop_speed_thresh and remaining_dist < stop_dist_thresh and vehicle not in counted_vehicles:
                    counted_vehicles.append(vehicle)
                    self.lane_queue[tls_idx] += 1

    def update_pedestrian_queue(self):
        for crossing in self.configuration.crossings:
            for edge in crossing['walking_areas']:
                pedestrians = traci.edge.getLastStepPersonIDs(edge)

                # Check who is waiting at the crossing
                # We assume that pedestrians push the button upon standing still for 1s
                for ped in pedestrians:
                    if traci.person.getWaitingTime(ped) >= 1 and traci.person.getNextEdge(ped) in crossing['crossing']:
                        self.lane_queue[crossing['lane']] += 1

    def step(self):
        # Step through the simulation until self.max_iterations steps have completed
        if self.simulation_step <= self.max_iterations:
            traci.simulationStep()
            self.simulation_step += 1
            return True
        else:
            print(self.max_iterations, 'iterations complete. Ending Simulation.')
            sys.stdout.flush()
            traci.close()
            return False

    def arrival_prediction(self):
        arr_mat = np.zeros((self.prediction_horizon, self.configuration.num_signals))

        vehicles = np.unique(traci.vehicle.getIDList())

        for vehicle in vehicles:
            # Returns a list of tuples (tls_id, tls_index, distance, state)
            next_tls = traci.vehicle.getNextTLS(vehicle)
            if not next_tls:
                continue

            # We only care about the first in the list
            _, tls_lane_idx, remaining_distance, _ = next_tls[0]

            signal_lane_id = traci.trafficlight.getControlledLanes(self.configuration.tls_id)[tls_lane_idx]
            speed = traci.vehicle.getSpeed(vehicle)

            # Calculate the time until the car arrives at the light
            if speed > 0.0:
                time_until_tls = remaining_distance / speed

                k = int(time_until_tls // self.time_step_len) - 1

                if 0 <= k < self.prediction_horizon:
                    arr_mat[k, self.lane_map[signal_lane_id]] += 1

        return arr_mat
