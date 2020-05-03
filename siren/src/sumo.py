import os 
import sys
import argparse
import random
import json
from sumolib import checkBinary
import traci
import numpy as np





class SUMO_Simulation:
    color_string_map = {"r": 0, "y": 2, "g": 1, "G": 1, "s": 2, "u": 3}

    def __init__(self, max_iterations = 10000,\
        config_file = '../SUMO/intersections/aarhus_intersection/osm.sumocfg',\
        gui_bin_loc = "/usr/bin/sumo-gui",\
        json_description_file="../SUMO/intersections/aarhus_intersection/osm.desc.json"):
        self.simulation_step = 0
        self.time_step_len = 1.0 # Temporary (seconds per timestep)
        self.prediction_horizon = 10 # Temporary
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

        self.options = self.arguments()
        
        if self.options.nogui:
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
        parser.add_argument("--nogui", action="store_true", help="run the commandline version of sumo")
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
        signal_lanes = traci.trafficlight.getControlledLanes(self.tlsID)
        self.lane_map = {}
        for i, lane in enumerate(signal_lanes):
            self.lane_map[lane] = self.lane_mapping_vec[i]

        
    def generate_route_file(self):
        print("Generating Route File.")
        def route_string(route_id, edges):
            r_str = "<route id=\"{}\" edges=\"{}\" />".format(route_id, edges)
        

    def start_sim(self):
        # Initiate the Traci GUI
        print("Starting Sumo Simulation.")
        traci.start(self.sumoCmd)
        self.map_lanes_to_signals()
        self.num_tls_lanes = len(traci.trafficlight.getControlledLanes(self.tlsID))
        
    def get_lights(self):
        light_mat = np.zeros((self.num_signals, 4))
        lane_light_state = traci.trafficlight.getRedYellowGreenState(self.tlsID)
        for lane_idx, light in enumerate(lane_light_state):
            light_mat[self.lane_mapping_vec[lane_idx], self.color_string_map[light]] = 1
        print(lane_light_state)
        print(light_mat)
        return light_mat
    
    def set_lights(self, light_marix=None):
        for signal_idx in range(self.num_signals):
            for color_idx in range(4):
                cur_light_state = light_marix[signal_idx, color_idx]
                print(np.where(self.lane_mapping_vec == signal_idx))
        # TODO: Finish this. 
    
    def get_light_times(self):
        return self.light_times


    def update_light_times(self):
        if self.light_times is None:
            self.light_times = np.zeros((self.num_tls_lanes, 4))
            self.prev_light_state = " " * self.num_tls_lanes
        current_light_state = traci.trafficlight.getRedYellowGreenState(self.tlsID)
        for i, light in enumerate(current_light_state):
            light_mat_idx = self.color_string_map[light]
            # print(i, light_mat_idx)
            if light == self.prev_light_state[i]:
                self.light_times[i, light_mat_idx] += 1
            else:
                self.light_times[i, light_mat_idx] = 0
        # print(self.light_times, "\n\n")
        self.prev_light_state = current_light_state

    def get_queue(self):
        return self.queue

    def update_queue(self, stop_speed_thresh=3., stop_dist_thresh=10.):
        self.queue = np.zeros((self.num_tls_lanes, 1))
        tls_lane_ids = traci.trafficlight.getControlledLanes(self.tlsID)
        # print(tls_lane_ids)
        counted_vehicles = []
        for tls_idx, tls_lane in enumerate(tls_lane_ids):
            for vehicle in np.unique(traci.lane.getLastStepVehicleIDs(tls_lane)):
                # print(vehicle, traci.vehicle.isStopped(vehicle))
                speed = traci.vehicle.getSpeed(vehicle)
                remaining_dist = traci.vehicle.getNextTLS(vehicle)[0][2]
                if speed < stop_speed_thresh and remaining_dist < stop_dist_thresh and not vehicle in counted_vehicles:
                    counted_vehicles.append(vehicle)
                    # print("Vehicle stopped: ", vehicle, tls_lane)
                    self.queue[tls_idx] += 1
                    # signal_lane_id = traci.trafficlight.getControlledLanes(self.tlsID)[tls_lane_idx]
        # print(self.queue)

        signal_queue = np.zeros((self.num_signals, 1))
        for i, lane_queue in enumerate(self.queue):
            # print(i, lane_queue, self.lane_mapping_vec[i])
            signal_queue[self.lane_mapping_vec[i]] += lane_queue
    
        return signal_queue
    
    def step_sim(self, light_matrix):

        # Step through the simulation until self.max_iterations steps have completed
        arr_cars = self.arrival_prediction()
        self.update_queue()
        self.update_light_times()
        
        
                
        # print(traci.trafficlight.getControlledLanes(tlsID))
        # print(traci.trafficlight.getControlledLinks(tlsID))
        # print(traci.trafficlight.getPhase(tlsID))
        # traci.trafficlight.setRedYellowGreenState(tlsID, "GGGGGGGGGGGrGGGGG")
        # print(data)
        if self.simulation_step <= self.max_iterations:
            traci.simulationStep()
            self.simulation_step += 1
            return True
        else:
            print(self.max_iterations, "iterations complete. Ending Simulation.")
            traci.close()
            sys.stdout.flush()
            return False
    def signal_to_lane_idx_map(self):
        pass




    def arrival_prediction(self):
        arr_mat = np.zeros((self.prediction_horizon, self.num_signals)).astype(int)
        # Loop through all lanes in the intersection
        vehicles = np.unique(traci.vehicle.getIDList())
        for k in range(self.prediction_horizon):
            time_window_len = (k+1) * self.time_step_len

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
                        vehicles = np.delete(vehicles, np.where(vehicles==vehicle))
                        arr_mat[k, self.lane_map[signal_lane_id]] += 1
        return arr_mat
