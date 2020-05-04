import argparse
import timeit
import time

import numpy as np

from gurobi.model import GurobiIntersection
from gurobi.intersections import SuperSimpleArrival, super_simple_configuration
from gurobi.structures import Options, Configuration, PerLaneDeparture, ConstantDeparture

from sumo.sumo import SUMOSimulation


def test(args):
    def optimize_wrapper():
        model = GurobiIntersection(super_simple_configuration, Options())

        departure = ConstantDeparture()
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
    model.iis(np.array([3, 12]), SuperSimpleArrival(), ConstantDeparture())


def sumo(args):
    sim = SUMOSimulation(args)
    configuration = Configuration(**sim.get_configuration())
    model = GurobiIntersection(configuration, Options())
    sim.prediction_horizon = model.options.prediction_horizon

    departure = PerLaneDeparture(sim.departure_rate)

    print("Sumo Object created")
    sim.start()

    sim_continue_flag = sim.step()

    while sim_continue_flag:
        queue = sim.get_queue()
        print("Queue: {}".format(queue))

        arr = sim.arrival_prediction()

        for i in range(20):
            t1 = time.time()
            sim_continue_flag = sim.step()
            t2 = time.time()

            if not sim_continue_flag:
                return False

            if t2 - t1 < 0.05:
                time.sleep(0.05 - (t2 - t1))

        light_matrix = model.optimize(queue, arr, departure, verbose=args.verbose)

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
