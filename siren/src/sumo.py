import os 
import sys
import optparse
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
        self.max_iterations = max_iterations
        self.json_description_file = json_description_file
        self.parse_intersection_description()
        if "SUMO_HOME" in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            sys.exit("SUMO_HOME not declared in path.")
        optParser = optparse.OptionParser()
        optParser.add_option("--nogui", action="store_true",
                            default=False, help="run the commandline version of sumo")
        self.options, self.option_args = optParser.parse_args()
        
        if self.options.nogui:
            self.sumoBinary = checkBinary("sumo")
            print("No GUI detected.")
        else:
            self.sumoBinary = checkBinary("sumo-gui")
            print("GUI detected.")
        self.sumoCmd = [self.sumoBinary, "-c", config_file, "--tripinfo-output", "tripinfo.xml"]
        self.parse_intersection_description()
    
    def parse_intersection_description(self):
        # Open the description.json file and save all values to the class
        with open(self.json_description_file) as file:
            desc_data = json.loads(file.read())

        # Convert JSON keys to class member names
        self.intersection_name = desc_data["intersection_name"]
        self.num_lights = desc_data["num_lights"]
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
        print(self.lane_map)

        
    def generate_route_file(self):
        print("Generating Route File.")
        def route_string(route_id, edges):
            r_str = "<route id=\"{}\" edges=\"{}\" />".format(route_id, edges)
        

    def start_sim(self):
        # Initiate the Traci GUI
        print("Starting Sumo Simulation.")
        traci.start(self.sumoCmd)
        self.map_lanes_to_signals()
        self.counted_vehicle_ids = []
        
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




    def arrival_prediction(self, s=None, k=1):
        time_window_len = k * self.time_step_len
        arr_vec = np.zeros((self.num_lights, 1)).astype(int)
        # Loop through all lanes in the intersection
        lanes = []
        for lane in traci.trafficlight.getControlledLanes(self.tlsID):
            # Loop through vehicles in that lane
            if not lane in lanes:
                lanes.append(lane)
                traci.lane.getLastStepVehicleIDs(lane)
                for vehicle in np.unique(traci.lane.getLastStepVehicleIDs(lane)):

                    vehicle_speed = traci.vehicle.getSpeed(vehicle)
                    nextTLS = traci.vehicle.getNextTLS(vehicle)[0]
                    # Make sure it's the correct traffic signal
                    # print(vehicle)
                    if nextTLS[0] == self.tlsID:
                        distance = nextTLS[2]
                        if time_window_len * vehicle_speed >= distance:
                            print("Arriving car: ", vehicle,  lane)
                            # print(self.lane_map[lane])
                            if not vehicle in self.counted_vehicle_ids:
                                self.counted_vehicle_ids.append(vehicle)
                                arr_vec[self.lane_map[lane]]+= 1
                    # print("\n")
        print(arr_vec)