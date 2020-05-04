import argparse
import timeit

import numpy as np

from gurobi.model import GurobiIntersection
from gurobi.intersections import SuperSimpleArrival, \
    SuperSimpleDeparture, super_simple_configuration
from gurobi.structures import Options, Configuration

from sumo.sumo import SUMOSimulation


def test(args):
    def optimize_wrapper():
        sim = SUMOSimulation(args)
        configuration = Configuration(**sim.get_configuration())
        model = GurobiIntersection(configuration, Options())

        departure = SuperSimpleDeparture()
        arrival = SuperSimpleArrival()
        queue = np.array([0, 0])

        for i in range(args.iterations):
            lights = model.optimize(queue, arrival, departure, verbose=args.verbose)

            for s in range(super_simple_configuration.num_signals):
                queue[s] += arrival[1, s] - departure[1, s] * lights[s, GurobiIntersection.GREEN]
                if queue[s] < 0:
                    queue[s] = 0

    print("Execution took {}s".format(timeit.timeit(optimize_wrapper, number=1)))


def iis():
    model = GurobiIntersection(super_simple_configuration, Options())
    model.iis(np.array([3, 12]), SuperSimpleArrival(), SuperSimpleDeparture())


def sumo(args):
    sim = SUMOSimulation(args)
    configuration = Configuration(**sim.get_configuration())
    model = GurobiIntersection(configuration, Options())
    sim.prediction_horizon = model.options.prediction_horizon

    arrival = SuperSimpleArrival()
    departure = SuperSimpleDeparture()

    print("Sumo Object created")
    sim.start()

    sim_continue_flag = True
    while sim_continue_flag:
        sim_continue_flag = sim.step()
        queue = sim.get_queue()
        print("Queue: {}".format(queue))

        arr = sim.arrival_prediction()
        light_matrix = model.optimize(queue, arr, departure, verbose=args.verbose)
        # for s in range(super_simple_configuration.num_signals):
        #     queue[s] += arr[1, s] - departure[1, s] * light_matrix[s, GurobiIntersection.GREEN]
        #     if queue[s] < 0:
        #         queue[s] = 0
        sim.set_lights(light_matrix)


def parse_arguments():
    def iteration_type(x):
        x = int(x)
        if x < 1:
            raise argparse.ArgumentTypeError("Minimum iterations is 1")
        return x

    parser = argparse.ArgumentParser(description='Test siren performance.')

    actions = parser.add_mutually_exclusive_group(required=False)
    actions.add_argument('-d', '--debug', dest='iis', action='store_true',
                         help='Store irreducible inconsistent system. Requires that model is infeasible.')
    actions.add_argument('-s', '--sumo', action='store_true', help='Run sumo simulation with our controller.')

    parser.add_argument("--nogui", action="store_true", help="run the commandline version of sumo")

    parser.add_argument('-v', '--verbose', action='store_true', help='Print variables after execution.')
    parser.add_argument('-i', '--iterations', type=iteration_type, default=1, help='Number of iterations to test over.')

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.sumo:
        sumo(args)
    elif args.iis:
        iis()
    else:
        test(args)


if __name__ == "__main__":
    main()
