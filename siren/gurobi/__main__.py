import argparse
import timeit

from gurobi.model import Intersection
from gurobi.intersections import super_simple_init as init, SuperSimpleArrival as arrival, \
    SuperSimpleDeparture as departure, super_simple_configuration as configuration
from gurobi.structures import Options


def test(args):
    model = Intersection(configuration, Options())

    def optimize_wrapper():
        model.optimize(init, arrival(), departure(), verbose=args.verbose)

    print("Execution took {}s".format(timeit.timeit(optimize_wrapper, number=args.iterations)))


def colors(args):
    model = Intersection(configuration, Options())
    model.optimize(init, arrival(), departure(), verbose=args.verbose)

    print('The next colors are:')
    print(model.get_colors())


def iis():
    model = Intersection(configuration, Options())
    model.iis(init, arrival(), departure())


def parse_arguments():
    def iteration_type(x):
        x = int(x)
        if x < 1:
            raise argparse.ArgumentTypeError("Minimum iterations is 1")
        return x

    parser = argparse.ArgumentParser(description='Test siren performance')

    actions = parser.add_mutually_exclusive_group(required=False)
    actions.add_argument('-s', '--store-iis', dest='iis', action='store_true',
                         help='Store irreducible inconsistent system. Requires that model is infeasible.')
    actions.add_argument('-c', '--colors', action='store_true', help='Print next colors from the last optimization')

    parser.add_argument('-v', '--verbose', action='store_true', help='Print variables after execution')
    parser.add_argument('-i', '--iterations', type=iteration_type, default=1, help='Number of iterations to test over')

    return parser.parse_args()


def main():
    args = parse_arguments()

    if args.colors:
        colors(args)
    elif args.iis:
        iis()
    else:
        test(args)


if __name__ == "__main__":
    main()
