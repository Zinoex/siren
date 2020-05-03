import os
import sys
import argparse
import json
from sumolib import checkBinary
import traci
import numpy as np


class SUMOSimulation:
    color_string_map = {"r": 1, "y": 2, "g": 0, "G": 0, "s": 2, "u": 3}
    color_string_map_reverse_list = ["G", "r", "y", "a"]

    def __init__(self, options, max_iterations=10000,
                 config_file='../sumo/intersections/aarhus_intersection/osm.sumocfg',
                 json_description_file="../sumo/intersections/aarhus_intersection/osm.desc.json"):
        self.simulation_step = 0
        self.time_step_len = 1.0  # Temporary (seconds per timestep)
        self.prediction_horizon = 10  # Temporary
        self.max_iterations = max_iterations
        self.json_description_file = json_description_file
        self.parse_intersection_description()
        self.queue = None
        self.light_times = None
        self.lights = None
        if "SUMO_HOME" in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            sys.exit("SUMO_HOME not declared in path.")

        if options.nogui:
            self.sumoBinary = checkBinary("sumo")
            print("No GUI detected.")
        else:
            self.sumoBinary = checkBinary("sumo-gui")
            print("GUI detected.")
        self.sumoCmd = [self.sumoBinary, "-c", config_file, "--tripinfo-output", "tripinfo.xml"]
        self.parse_intersection_description()

    @staticmethod
    def arguments():
        parser = argparse.ArgumentParser('Test performance or run simulations of siren')
        return parser.parse_args()

    def parse_intersection_description(self):
        # Open the description.json file and save all values to the class
        with open(self.json_description_file) as file:
            desc_data = json.loads(file.read())

        # Convert JSON keys to class member names
        self.intersection_name = desc_data["intersection_name"]
        self.num_signals = desc_data["num_signals"]
        self.tlsID = desc_data["traffic_junction_id"]
        self.config_dict = {}
        self.config_dict["num_signals"] = self.num_signals
        self.config_dict["conflict_matrix"] = np.array(desc_data["conflict_matrix"])
        self.config_dict["green_interval"] = np.array(desc_data["green_interval_matrix"])
        self.config_dict["yellow_time"] = np.array(desc_data["time_vectors"]["yellow"])
        self.config_dict["amber_time"] = np.array(desc_data["time_vectors"]["amber"])
        self.config_dict["min_green"] = np.array(desc_data["time_vectors"]["amber"])

        self.lane_mapping_vec = desc_data["SUMO_lane_mapping"]

    def get_configuration(self):
        return self.config_dict

    def map_lanes_to_signals(self):
        # Create lane_map dictionary to turn lanes to signals
        signal_lanes = traci.trafficlight.getControlledLanes(self.tlsID)
        self.lane_map = {}
        for i, lane in enumerate(signal_lanes):
            self.lane_map[lane] = self.lane_mapping_vec[i]

    def start_sim(self):
        # Initiate the Traci GUI
        print("Starting Sumo Simulation.")
        traci.start(self.sumoCmd)
        # Now that traci is started we can get lane ids
        self.map_lanes_to_signals()
        # Constant number of lanes is used a lot
        self.num_tls_lanes = len(traci.trafficlight.getControlledLanes(self.tlsID))
        # Set lights to off (default state)
        traci.trafficlight.setRedYellowGreenState(self.tlsID, "o" * self.num_tls_lanes)

    def get_lights(self):
        # Initialize matrix to zeros
        light_mat = np.zeros((self.num_signals, 4))
        # Get the current light state
        lane_light_state = traci.trafficlight.getRedYellowGreenState(self.tlsID)
        # Loop through each lane and update the signal matrix by the correct value
        for lane_idx, light in enumerate(lane_light_state):
            light_mat[self.lane_mapping_vec[lane_idx], self.color_string_map[light]] = 1
        return light_mat

    def set_lights(self, light_matrix):
        # Set default as off (could be red too) to allow indexing of the list
        output_light_list = ["o"] * self.num_tls_lanes
        
        # Argmax gives which light is a 1 for each signal
        # Assumes light rows are one-hot-encoded
        for i, color_idx in enumerate(np.argmax(light_matrix, axis=1)):
            # Loop through to find all lane mappings for each signal
            for ii in [k for k, v in self.lane_mapping_vec if v == i]:
                output_light_list[ii] = self.color_string_map_reverse_list[color_idx]
        traci.trafficlight.setRedYellowGreenState(self.tlsID, "".join(output_light_list))

    def get_light_times(self):
        return self.light_times

    def update_light_times(self):
        # Not sure if this if statement still needs to be there, was for debugging
        if self.light_times is None:
            self.light_times = np.zeros((self.num_tls_lanes, 4))
            self.prev_light_state = " " * self.num_tls_lanes
        # Get current light state from traci (per lane)
        current_light_state = traci.trafficlight.getRedYellowGreenState(self.tlsID)
        # Loop through all lanes
        for i, light in enumerate(current_light_state):
            # Map each lane to a signal
            light_mat_idx = self.color_string_map[light]
            # Increment if it's the same
            if light == self.prev_light_state[i]:
                self.light_times[i, light_mat_idx] += 1
            # Reset if it changed
            else:
                self.light_times[i, light_mat_idx] = 0
        self.prev_light_state = current_light_state

    def get_queue(self):
        return self.queue

    def update_queue(self, stop_speed_thresh=3., stop_dist_thresh=10.):
        # Initialize to zero (queue is in terms of lanes)
        self.queue = np.zeros((self.num_tls_lanes, 1))
        # Get list of all lane ids connected to the traffic light
        tls_lane_ids = traci.trafficlight.getControlledLanes(self.tlsID)
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
                if speed < stop_speed_thresh and remaining_dist < stop_dist_thresh and not vehicle in counted_vehicles:
                    # count it
                    counted_vehicles.append(vehicle)
                    # increase the queue
                    self.queue[tls_idx] += 1
        # self.queue was described using tls lanes, we want signals
        # Initialize to zero
        signal_queue = np.zeros((self.num_signals, 1))
        # Loop through queue just generated
        for i, lane_queue in enumerate(self.queue):
            # Increment the signal queue this lane belongs to by all cars on the lane
            signal_queue[self.lane_mapping_vec[i]] += lane_queue

        return signal_queue

    def step_sim(self):
        # Step through the simulation until self.max_iterations steps have completed
        arr_cars = self.arrival_prediction()
        # Do this every step
        self.update_queue()
        # Run simulation until max_iterations
        if self.simulation_step <= self.max_iterations:
            traci.simulationStep()
            self.simulation_step += 1
            return True
        else:
            print(self.max_iterations, "iterations complete. Ending Simulation.")
            traci.close()
            sys.stdout.flush()
            return False

    def arrival_prediction(self):
        # Initialize the array
        arr_mat = np.zeros((self.prediction_horizon, self.num_signals)).astype(int)
        # Get all vehicles in model
        vehicles = np.unique(traci.vehicle.getIDList())
        for k in range(self.prediction_horizon):
            # k starts at zero (we want a time for the first index of self.time_step_len)
            time_window_len = (k + 1) * self.time_step_len
            # Loop through the vehicles (this array has elements deleted later in this function to not double count)
            for vehicle in vehicles:
                nextTLS = traci.vehicle.getNextTLS(vehicle)
                if nextTLS:
                    # Returns a tuple, but we only care about the first in the list
                    nextTLS = nextTLS[0]
                    # Lane ID in the traffic light signal
                    tls_lane_idx = nextTLS[1]
                    signal_lane_id = traci.trafficlight.getControlledLanes(self.tlsID)[tls_lane_idx]
                    # Distance remaining to the traffic light
                    remaining_distance = nextTLS[2]
                    # Current Vehicle Speed
                    speed = traci.vehicle.getSpeed(vehicle)
                    # Calculate the time until the car arrives at the light
                    if speed > 0.0:
                        time_until_tls = remaining_distance / speed
                    else:
                        # The vehicle isn't moving, we can't predict it's arrival
                        time_until_tls = np.inf
                    if time_until_tls < time_window_len:
                        # Delete the vehicle so it's not counted again
                        vehicles = np.delete(vehicles, np.where(vehicles == vehicle))
                        arr_mat[k, self.lane_map[signal_lane_id]] += 1
        return arr_mat
