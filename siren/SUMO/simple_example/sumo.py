import os 
import sys
import optparse
import random
import json
from sumolib import checkBinary
import traci




class SUMO_Simulation:
    def __init__(self, max_iterations = 10000,\
        config_file = '../intersections/aarhus_intersection/osm.sumocfg',\
        gui_bin_loc = "/usr/bin/sumo-gui",\
        json_description_file="../intersections/aarhus_intersection/osm.desc.json"):
        self.simulation_step = 0
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
    
    def parse_intersection_description(self):
        # Open the description.json file and save all values to the class
        with open(self.json_description_file) as file:
            desc_data = json.loads(file.read())

        # Convert JSON keys to class member names
        for key in desc_data.keys():
            exec("self.%s = %s" % (key, 'desc_data[key]'))
    
    def generate_route_file(self):
        print("Generating Route File.")
        def route_string(route_id, edges):
            r_str = "<route id=\"{}\" edges=\"{}\" />".format(route_id, edges)
           
        
    def start_sim(self):
        # Initiate the Traci GUI
        print("Starting Sumo Simulation.")
        traci.start(self.sumoCmd)
        
    def step_sim(self):
        # Step through the simulation until self.max_iterations steps have completed
        tlsID = "gneJ29"
        # print(traci.trafficlight.getControlledLanes(tlsID))
        # print(traci.trafficlight.getControlledLinks(tlsID))
        # print(traci.trafficlight.getPhase(tlsID))
        traci.trafficlight.setRedYellowGreenState(tlsID, "rrrrrrrrrrrrrrrrr")
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

    def update_lights(self):
        pass
    def get_traffic_info():
        pass
