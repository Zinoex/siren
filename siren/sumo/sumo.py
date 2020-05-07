import os
import sys
from sumolib import checkBinary
import traci
import numpy as np


class SUMOSimulation:
    color_string_map = {'r': 1, 'y': 2, 'g': 0, 'G': 0, 's': 2, 'u': 3}
    color_string_map_reverse_list = ['g', 'r', 'y', 'u']

    def __init__(self, configuration, options):
        self.simulation_step = 0
        self.time_step_len = 1.0  # Temporary (seconds per timestep)

        self.prediction_horizon = options.prediction_horizon
        self.max_iterations = options.iterations
        self.configuration = configuration
        self.options = options

        self.queue = None
        self.lane_map = {}
        self.num_tls_lanes = 0

        self.test_sumo_home()

        self.sumo_cmd = [self.sumo_binary(options),
                         '-c', self.configuration.config_file,
                         '--tripinfo-output', 'data/{}/tripinfo.xml'.format(options.folder_prefix),
                         '--summary', 'data/{}/summary.xml'.format(options.folder_prefix),
                         '--device.emissions.probability', '1',
                         '--seed', '42',
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
        for i, lane in enumerate(signal_lanes):
            self.lane_map[lane] = self.configuration.lane_mapping_vec[i]

    def start(self):
        # Initiate the Traci GUI
        print('Starting Sumo Simulation.')
        traci.start(self.sumo_cmd)
        # Now that traci is started we can get lane ids
        self.map_lanes_to_signals()
        # Constant number of lanes is used a lot
        self.num_tls_lanes = len(traci.trafficlight.getControlledLanes(self.configuration.tls_id))

        if not self.options.timed:
            # Set lights to off (default state)
            traci.trafficlight.setRedYellowGreenState(self.configuration.tls_id, 'o' * self.num_tls_lanes)

    def get_lights(self):
        # Initialize matrix to zeros
        light_mat = np.zeros((self.configuration.num_signals, 4))
        # Get the current light state
        lane_light_state = traci.trafficlight.getRedYellowGreenState(self.configuration.tls_id)
        # Loop through each lane and update the signal matrix by the correct value
        for lane_idx, light in enumerate(lane_light_state):
            light_mat[self.configuration.lane_mapping_vec[lane_idx], self.color_string_map[light]] = 1
        return light_mat

    def set_lights(self, light_matrix):
        # Set default as off (could be red too) to allow indexing of the list
        output_light_list = ['o'] * self.num_tls_lanes
        # Argmax gives which light is a 1 for each signal
        # Assumes light rows are one-hot-encoded
        for i, color_idx in enumerate(np.argmax(light_matrix, axis=1)):
            # Loop through to find all lane mappings for each signal
            for ii, sig_idx in enumerate(self.configuration.lane_mapping_vec):
                if sig_idx == i:
                    # Update appropriate light
                    output_light_list[ii] = self.color_string_map_reverse_list[color_idx]
        traci.trafficlight.setRedYellowGreenState(self.configuration.tls_id, ''.join(output_light_list))

    def get_queue(self):
        return self.queue

    def update_queue(self, stop_speed_thresh=3., stop_dist_thresh=100.):
        # Initialize to zero (queue is in terms of lanes)
        lane_queues = np.zeros(self.num_tls_lanes)
        # Get list of all lane ids connected to the traffic light
        tls_lane_ids = traci.trafficlight.getControlledLanes(self.configuration.tls_id)
        # This is to not double count vehicles (might be a little hacky)
        counted_vehicles = []
        # Loop through lanes
        for tls_idx, tls_lane in enumerate(tls_lane_ids):
            # Loop through every vehicle on the lane
            for vehicle in np.unique(traci.lane.getLastStepVehicleIDs(tls_lane)):
                # Get the speed from the vehicle class
                speed = traci.vehicle.getSpeed(vehicle)
                # getNextTLS returns a list of lists
                # first index is which light is next (we only care about next)
                # second index is for distance to light
                remaining_dist = traci.vehicle.getNextTLS(vehicle)[0][2]
                # If it's going slow enough && it's close enough && we haven't counted it
                if speed < stop_speed_thresh and remaining_dist < stop_dist_thresh and vehicle not in counted_vehicles:
                    # count it
                    counted_vehicles.append(vehicle)
                    # increase the queue
                    lane_queues[tls_idx] += 1
        # self.queue was described using tls lanes, we want signals
        # Initialize to zero
        self.queue = np.zeros(self.configuration.num_signals)
        # Loop through queue just generated
        for i, lane_queue in enumerate(lane_queues):
            # Increment the signal queue this lane belongs to by all cars on the lane
            self.queue[self.configuration.lane_mapping_vec[i]] += lane_queue

    def step(self):
        # Step through the simulation until self.max_iterations steps have completed
        # arr_cars = self.arrival_prediction()
        # Do this every step
        self.update_queue()
        # Run simulation until max_iterations
        if self.simulation_step <= self.max_iterations:
            traci.simulationStep()
            self.simulation_step += 1
            return True
        else:
            print(self.max_iterations, 'iterations complete. Ending Simulation.')
            traci.close()
            sys.stdout.flush()
            return False

    def arrival_prediction(self):
        # Initialize the array
        arr_mat = np.zeros((self.prediction_horizon, self.configuration.num_signals)).astype(int)

        # Get all vehicles in model
        vehicles = np.unique(traci.vehicle.getIDList())

        # Loop through the vehicles (this array has elements deleted later in this function to not double count)
        for vehicle in vehicles:
            next_tls = traci.vehicle.getNextTLS(vehicle)
            if next_tls:
                # Returns a tuple, but we only care about the first in the list
                next_tls = next_tls[0]
                # Lane ID in the traffic light signal
                tls_lane_idx = next_tls[1]
                signal_lane_id = traci.trafficlight.getControlledLanes(self.configuration.tls_id)[tls_lane_idx]
                # Distance remaining to the traffic light
                remaining_distance = next_tls[2]
                # Current Vehicle Speed
                speed = traci.vehicle.getSpeed(vehicle)
                # Calculate the time until the car arrives at the light
                if speed > 0.0:
                    time_until_tls = remaining_distance / speed

                    k = int(time_until_tls // self.time_step_len) - 1

                    if 0 <= k < self.prediction_horizon:
                        arr_mat[k, self.lane_map[signal_lane_id]] += 1

        return arr_mat
