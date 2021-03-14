import argparse
import re
import timeit
from typing import Tuple

import numpy as np

from gurobi.model import GurobiIntersection
from gurobi.structures import GurobiOptions, ConstantDeparture, ConstantArrival
from sumo.configuration_loader import ConfigurationLoader
from sumo.runners import TimedRunner, MPCRunner


def positive_type(x):
    x = int(x)
    if x < 1:
        raise argparse.ArgumentTypeError('{} is an invalid positive integer value.'.format(x))
    return x


def folder_prefix(x):
    x = str(x)
    if not re.match('^[{}a-z0-9_]+$', x):
        raise argparse.ArgumentTypeError('Parameter must be alphanumeric and underscores only.')
    return x


def range_or_int(x):
    x = str(x)
    if not re.match('^(\d+:\d+|\d+)$', x):
        raise argparse.ArgumentTypeError('Parameter must be a range (\d+:\d+) or an integer')

    if ':' in x:
        start, end = x.split(':')
        start, end = int(start), int(end)

        if start < 1:
            raise argparse.ArgumentTypeError('Start must be greater than 0.')

        if end < 1:
            raise argparse.ArgumentTypeError('Start must be greater than 0.')

        if end < start:
            raise argparse.ArgumentTypeError('End must be greater than start.')

        return start, end
    else:
        x = int(x)

        if x < 1:
            raise argparse.ArgumentTypeError('{} is an invalid positive integer value.'.format(x))

        return x


def prediction_control_validity_check(parser: argparse.ArgumentParser, args):
    if isinstance(args.prediction_horizon, tuple) and isinstance(args.control_horizon, tuple):
        pstart, pend = args.prediction_horizon
        cstart, cend = args.control_horizon
        if pstart != cstart or pend != cend:
            parser.error('Ranges for prediction horizon and control horizon do not match')
    elif isinstance(args.prediction_horizon, tuple) and isinstance(args.control_horizon, int):
        pstart, pend = args.prediction_horizon
        if pstart < args.control_horizon:
            parser.error('All prediction horizons must be greater than or equal to the control horizon')
    elif isinstance(args.prediction_horizon, int) and isinstance(args.control_horizon, tuple):
        cstart, cend = args.control_horizon
        # Subtract one to account for the exclusive upper bound of range
        if cend - 1 > args.prediction_horizon:
            parser.error('The prediction horizon must be greater than or equal to all control horizons')
    else:
        if args.control_horizon > args.prediction_horizon:
            parser.error('The prediction horizon must be greater than or equal to the control horizon')


def benchmarking(args):
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

    print('Execution took {:.3f}s'.format(timeit.timeit(optimize_wrapper, number=args.repeat)))


def iis(args):
    configuration_loader = ConfigurationLoader(args.name)

    options = GurobiOptions(**vars(args))
    model = GurobiIntersection(configuration_loader.gurobi_config, options)
    model.iis(np.zeros(configuration_loader.num_signals), ConstantArrival(), ConstantDeparture())


def sumo(args):
    runner = TimedRunner(args) if args.timed else MPCRunner(args)
    runner.run()


def range_or_int_length(x):
    if isinstance(x, tuple):
        start, end = x
        return end - start
    else:
        return 1


def expand_range(x, length):
    if isinstance(x, tuple):
        start, end = x
        return range(start, end)
    else:
        return [x for _ in range(length)]


def batch(args):
    prediction_horizon = args.prediction_horizon
    control_horizon = args.control_horizon

    length = max(range_or_int_length(prediction_horizon), range_or_int_length(control_horizon))

    prediction_horizon = expand_range(prediction_horizon, length)
    control_horizon = expand_range(control_horizon, length)

    for p, c in zip(prediction_horizon, control_horizon):
        args.prediction_horizon = p
        args.control_horizon = c

        print('Running simulation for prediction horizion = {}, control horizon = {}'.format(p, c))
        runner = TimedRunner(args) if args.timed else MPCRunner(args)
        runner.run()


def parse_arguments():
    parser = argparse.ArgumentParser(description='Test siren performance.')
    parser.add_argument('--timed', action='store_true', help='Run sumo simulation with timed control.')
    parser.add_argument('--queue-weight', type=float, default=1, help='Set queue weight.')
    parser.add_argument('--stops-weight', type=float, default=6, help='Set stops weight.')
    parser.add_argument('--wait-weight', type=float, default=0.05, help='Set wait weight (should be small).')
    parser.add_argument('--green-weight', type=float, default=0.01, help='Set green weight (should be small).')
    parser.add_argument('--throughput-weight', type=float, default=2, help='Set throughput weight.')

    parser.add_argument('-v', '--verbose', action='store_true', help='Print variables after execution.')
    parser.add_argument('-i', '--iterations', type=positive_type, default=1, help='Number of iterations to test over.')
    parser.add_argument('-f', '--folder-prefix', type=folder_prefix, default='experiment', help='Folder prefix for experiments.')
    parser.add_argument('-n', '--name', type=str, default='aarhus_intersection/osm', help='Intersetion to simulate.')

    commands = parser.add_subparsers(help='commands', dest='command')

    iis_parser = commands.add_parser("iis", help='Compute irreducible inconsistent system (gurobi only, no sumo). Requires that model is infeasible.')
    iis_parser.add_argument('-p', '--prediction-horizon', type=positive_type, default=30, help='Set prediction horizon.')
    iis_parser.add_argument('-c', '--control-horizon', type=positive_type, default=20, help='Set control horizon')
    iis_parser.set_defaults(func=iis)

    sumo_parser = commands.add_parser("sumo", help='Run sumo simulation with the timed or siren controller.')
    sumo_parser.add_argument('-p', '--prediction-horizon', type=positive_type, default=30, help='Set prediction horizon.')
    sumo_parser.add_argument('-c', '--control-horizon', type=positive_type, default=20, help='Set control horizon')
    sumo_parser.add_argument('--nogui', action='store_true', help='Run the commandline version of sumo.')
    sumo_parser.add_argument('--no-delay', action='store_true', help='Run sumo simulation without real-time delay.')
    sumo_parser.set_defaults(func=sumo)

    benchmarking_parser = commands.add_parser("benchmarking", help='Run the optimization of the siren control N times and output the time.')
    benchmarking_parser.add_argument('-p', '--prediction-horizon', type=positive_type, default=30, help='Set prediction horizon.')
    benchmarking_parser.add_argument('-c', '--control-horizon', type=positive_type, default=20, help='Set control horizon')
    benchmarking_parser.add_argument('-r', '--repeat', type=positive_type, default=1, help='Set the number of repetitions to compute the timing over')
    benchmarking_parser.set_defaults(func=benchmarking)

    batch_parser = commands.add_parser("batch", help='Run batches of sumo simulations with the timed or siren controller.')
    batch_parser.add_argument('-p', '--prediction-horizon', type=range_or_int, default=30, help='Set prediction horizon.')
    batch_parser.add_argument('-c', '--control-horizon', type=range_or_int, default=20, help='Set control horizon')
    batch_parser.set_defaults(func=batch, nogui=True, no_delay=True)

    return parser, parser.parse_args()


def main():
    parser, args = parse_arguments()
    prediction_control_validity_check(parser, args)

    args.func(args)


if __name__ == '__main__':
    main()
