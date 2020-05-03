from sumo import SUMO_Simulation as SUMO
import argparse
import sys
sys.path.append("../gurobi/")

from model import Intersection
from structures import Configuration
import argparse
import timeit


# from intersections import super_simple_init as init, super_simple_arrival_function as arrival_function, \
#     super_simple_departure_function as departure_function, super_simple_configuration as configuration
from structures import Options






def main():
    # opt_model = model = Intersection(configuration, Options(), arrival_function, departure_function)
    
    sumo_sim_obj = SUMO()
    configuration = Configuration(**sumo_sim_obj.get_configuration())
    
    # opt_model_obj = Intersection()
#    configuratio = sumo_sim_obj.configuration()

    sumo_sim_obj.generate_route_file()
    print("Sumo Object created")
    sumo_sim_obj.start_sim()

    sim_continue_flag = True
    while sim_continue_flag:
        sim_continue_flag =  sumo_sim_obj.step_sim()




if __name__ == "__main__":
    main()




# def main():
#     simple_intersection = Intersection("sample_itersection.json", "8 Lane, no lefts")
#     print(simple_intersection)

# if __name__ == "__main__":
#     main()
