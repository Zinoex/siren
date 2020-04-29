from sumo_sim import SUMO_Simulation


def main():
    sumo_sim_obj = SUMO_Simulation()
    sumo_sim_obj.start_sim()
    
    sim_continue_flag = True
    while sim_continue_flag:
        sim_continue_flag =  sumo_sim_obj.step_sim()

    
    # print("Compiled.")


# this is the main entry point of this script
if __name__ == "__main__":
    main()
    # options = get_options()
