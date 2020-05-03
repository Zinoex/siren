from sumo import SUMO_Simulation as SUMO
import argparse
import sys
sys.path.append("../gurobi/")

from model import Intersection





def main():
    # opt_model = model = Intersection(configuration, Options(), arrival_function, departure_function)
    
    sumo_sim_obj = SUMO()
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
