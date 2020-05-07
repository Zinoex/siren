import argparse
import json
import timeit
import time
import re
import os
from multiprocessing import Process, SimpleQueue

import numpy as np

from gurobi.model import GurobiIntersection
from gurobi.structures import GurobiOptions, PerLaneDeparture, ConstantDeparture, ConstantArrival
from sumo.configuration_loader import ConfigurationLoader

from sumo.sumo import SUMOSimulation


def test(args):
    configuration_loader = ConfigurationLoader(args.name)

    options = GurobiOptions(**vars(args))
    model = GurobiIntersection(configuration_loader.gurobi_config, options)

    departure = ConstantDeparture()
    arrival = ConstantArrival()

    def optimize_wrapper():
        queue = np.zeros(configuration_loader.num_signals)

        for i in range(args.iterations):
            lights = model.optimize(queue, arrival, departure, verbose=args.verbose)

            for s in range(configuration_loader.num_signals):
                queue[s] += arrival[1, s] - departure[1, s] * lights[s, GurobiIntersection.GREEN]
                if queue[s] < 0:
                    queue[s] = 0

    print('Execution took {}s'.format(timeit.timeit(optimize_wrapper, number=1)))


def iis(args):
    configuration_loader = ConfigurationLoader(args.name)

    options = GurobiOptions(**vars(args))
    model = GurobiIntersection(configuration_loader.gurobi_config, options)
    model.iis(np.zeros(configuration_loader.num_signals), ConstantArrival(), ConstantDeparture())


def sumo(args):
    os.makedirs('data/{}/'.format(args.folder_prefix), exist_ok=True)

    configuration_loader = ConfigurationLoader(args.name)
    sim = SUMOSimulation(configuration_loader.sumo_config, args)

    if not args.timed:
        options = GurobiOptions(**vars(args))
        model = GurobiIntersection(configuration_loader.gurobi_config, options)

        with open('data/{}/options.json'.format(args.folder_prefix), 'w') as options_file:
            options_file.write(json.dumps(options.__dict__))
            options_file.flush()

        departure = PerLaneDeparture(configuration_loader.departure_rate)

    print('Sumo Object created')
    sim.start()

    sim_continue_flag = sim.step()

    input_queue = SimpleQueue()
    output_queue = SimpleQueue()

    def optimize():
        with open('data/{}/timing.csv'.format(args.folder_prefix), 'w') as file:
            i = 0
            while sim_continue_flag:
                q, a = input_queue.get()

                t1 = time.time()
                lm = model.optimize(q, a, departure, verbose=args.verbose)
                t2 = time.time()
                file.write('{},{}\n'.format(i, t2 - t1))
                file.flush()

                output_queue.put(lm)
                i += 1

    if not args.timed:
        worker_process = Process(target=optimize)
        worker_process.start()

    try:
        while sim_continue_flag:
            queue = sim.get_queue()
            print('Queue: {}'.format(queue))

            arr = sim.arrival_prediction()
            if not args.timed:
                input_queue.put((queue, arr))

            for i in range(20):
                t1 = time.time()
                sim_continue_flag = sim.step()
                t2 = time.time()

                if not sim_continue_flag:
                    return False

                if not args.no_delay and t2 - t1 < 0.05:
                    time.sleep(0.05 - (t2 - t1))

            if not args.timed:
                light_matrix = output_queue.get()

                sim.set_lights(light_matrix)
    finally:
        if not args.timed:
            worker_process.kill()


def parse_arguments():
    def positive_type(x):
        x = int(x)
        if x < 1:
            raise argparse.ArgumentTypeError('{} is an invalid positive integer value.'.format(x))
        return x

    def alphanum_only(x):
        x = str(x)
        if not re.match('^[a-z0-9_]+$', x):
            raise argparse.ArgumentError('Parameter must be alphanumeric and underscores only.')
        return x

    parser = argparse.ArgumentParser(description='Test siren performance.')

    actions = parser.add_mutually_exclusive_group(required=False)
    actions.add_argument('-d', '--debug', dest='iis', action='store_true',
                         help='Store irreducible inconsistent system. Requires that model is infeasible.')
    actions.add_argument('-s', '--sumo', action='store_true', help='Run sumo simulation with our controller.')

    parser.add_argument('--nogui', action='store_true', help='Run the commandline version of sumo.')
    parser.add_argument('--timed', action='store_true', help='Run sumo simulation with timed control.')
    parser.add_argument('--no-delay', action='store_true', help='Run sumo simulation without real-time delay.')
    parser.add_argument('-p', '--prediction-horizon', type=positive_type, default=30, help='Set prediction horizon.')
    parser.add_argument('-c', '--control-horizon', type=positive_type, default=20, help='Set control horizon')
    parser.add_argument('--queue-weight', type=float, default=1, help='Set queue weight.')
    parser.add_argument('--stops-weight', type=float, default=6, help='Set stops weight.')
    parser.add_argument('--wait-weight', type=float, default=0.05, help='Set wait weight (should be small).')
    parser.add_argument('--green-weight', type=float, default=0.01, help='Set green weight (should be small).')
    parser.add_argument('--throughput-weight', type=float, default=2, help='Set throughput weight.')

    parser.add_argument('-v', '--verbose', action='store_true', help='Print variables after execution.')
    parser.add_argument('-i', '--iterations', type=positive_type, default=1, help='Number of iterations to test over.')
    parser.add_argument('-f', '--folder-prefix', type=alphanum_only, default='experiment',
                        help='Folder prefix for experiments.')
    parser.add_argument('-n', '--name', type=str, default='aarhus_intersection/osm', help='Intersetion to simulate.')

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.sumo:
        sumo(args)
    elif args.iis:
        iis(args)
    else:
        test(args)


if __name__ == '__main__':
    main()
