import os 
import sys
import argparse
import random
import json
from sumolib import checkBinary
import traci
import numpy as np




class SUMO_Simulation:
    def __init__(self, max_iterations = 10000,\
        config_file = '../intersections/aarhus_intersection/osm.sumocfg',\
        gui_bin_loc = "/usr/bin/sumo-gui",\
        json_description_file="../intersections/aarhus_intersection/osm.desc.json"):
        self.simulation_step = 0
        self.time_step_len = 1.0 # Temporary (seconds per timestep)
        self.prediction_horizon = 10 # Temporary
        self.max_iterations = max_iterations
        self.json_description_file = json_description_file
        self.parse_intersection_description()
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
        start_light_state = desc_data["start_light_state"]  
        time_vectors = desc_data["time_vectors"]
        green_interval_matrix = desc_data["green_interval_matrix"]
        conflict_matrix = desc_data["conflict_matrix"]

        self.lane_mapping_vec = desc_data["SUMO_lane_mapping"]

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
        
    def step_sim(self):
        # Step through the simulation until self.max_iterations steps have completed
        arr_cars = self.arrival_prediction()
        
                
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
