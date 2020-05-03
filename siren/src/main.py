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
from intersections import SuperSimpleDeparture as departure_function






def main():
    # opt_model = model = Intersection(configuration, Options(), arrival_function, departure_function)
    
    sumo_sim_obj = SUMO()
    configuration = Configuration(**sumo_sim_obj.get_configuration())
    g_model = Intersection(configuration, Options())
    sumo_sim_obj.prediction_horizon = g_model.options.prediction_horizon
    # opt_model_obj = Intersection()
#    configuratio = sumo_sim_obj.configuration()
    # sumo_sim_obj.generate_route_file()
    print("Sumo Object created")
    sumo_sim_obj.start_sim()

    sim_continue_flag = True
    while sim_continue_flag:
        sim_continue_flag =  sumo_sim_obj.step_sim()
        # light_times = sumo_sim_obj.get_light_times()
        queue = sumo_sim_obj.get_queue()
        print(queue)
        # light_state= sumo_sim_obj.get_lights()
        # sumo_sim_obj.set_lights()
        
        # init = Initialization(light_state, light_times, queue)
        arr = sumo_sim_obj.arrival_prediction()
        # print("Queue", queue)
        # print("Arrivals: ", arr)
        light_matrix = g_model.optimize(queue, arr, departure_function())
        # print(g_model.optimize(init, arr, departure_function(), verbose=True))
        
        # print("Light Matrix: ", light_matrix)
        sumo_sim_obj.set_lights(light_matrix)
        



if __name__ == "__main__":
    main()




# def main():
#     simple_intersection = Intersection("sample_itersection.json", "8 Lane, no lefts")
#     print(simple_intersection)

# if __name__ == "__main__":
#     main()
