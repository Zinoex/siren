import os
import sys
import optparse

from sumolib import checkBinary
import traci


class SUMOSimulation:
    def __init__(self, max_iterations=100, config_file='../intersections/aarhus_intersection/osm.sumocfg',\
         gui_bin_loc="/usr/bin/sumo-gui"):
        self.simulation_step = 0
        self.max_iterations = max_iterations

        if "SUMO_HOME" in os.environ:
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)
        else:
            sys.exit("SUMO_HOME not declared in path.")
        opt_parser = optparse.OptionParser()
        opt_parser.add_option("--nogui", action="store_true",
                             default=False, help="run the commandline version of sumo")

        self.options, self.option_args = opt_parser.parse_args()
        self.sumoCmd = [gui_bin_loc, "-c", config_file]
        
        if self.options.nogui:
            self.sumoBinary = checkBinary("sumo")
            print("No GUI detected.")
        else:
            self.sumoBinary = checkBinary("sumo-gui")
            print("GUI detected.")

    def start_sim(self):
        print("Starting Sumo Simulation.")
        traci.start(self.sumoCmd)
        # traci.trafficlight.setPhase("0", 2)

    def step_sim(self):
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

    def get_traffic_info(self):
        pass
