import argparse
import re
import timeit

import numpy as np

from gurobi.model import GurobiIntersection
from gurobi.structures import GurobiOptions, ConstantDeparture, ConstantArrival
from sumo.configuration_loader import ConfigurationLoader
from sumo.runners import TimedRunner, MPCRunner


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

    print('Execution took {:.3f}s'.format(timeit.timeit(optimize_wrapper, number=1)))


def iis(args):
    configuration_loader = ConfigurationLoader(args.name)

    options = GurobiOptions(**vars(args))
    model = GurobiIntersection(configuration_loader.gurobi_config, options)
    model.iis(np.zeros(configuration_loader.num_signals), ConstantArrival(), ConstantDeparture())


def sumo(args):
    runner = TimedRunner(args) if args.timed else MPCRunner(args)
    runner.run()


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
