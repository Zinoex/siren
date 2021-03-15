import json
import os
import time
from multiprocessing import SimpleQueue, Process

from gurobi.model import GurobiIntersection
from gurobi.structures import GurobiOptions, PerLaneDeparture
from sumo.configuration_loader import ConfigurationLoader
from sumo.sumo import SUMOSimulation


class Runner:
    def __init__(self, args):
        args.folder_prefix = args.folder_prefix.format(p=args.prediction_horizon, c=args.control_horizon)

        os.makedirs('data/{}/'.format(args.folder_prefix), exist_ok=True)

        self.configuration_loader = ConfigurationLoader(args.name)
        self.sim = SUMOSimulation(self.configuration_loader.sumo_config, args)
        self.args = args

    def run(self):
        raise NotImplemented()


class TimedRunner(Runner):
    def __init__(self, args):
        super().__init__(args)

    def run(self):
        print('Sumo Object created')
        self.sim.start()

        sim_continue_flag = self.sim.step()

        while sim_continue_flag:
            t1 = time.time()
            sim_continue_flag = self.sim.step()
            t2 = time.time()

            if not sim_continue_flag:
                return False

            if not self.args.no_delay and t2 - t1 < 0.05:
                time.sleep(0.05 - (t2 - t1))


class Optimizer(Process):
    def __init__(self, model, departure_rate, args):
        super().__init__()
        self.input = SimpleQueue()
        self.output = SimpleQueue()
        self.args = args
        self.model = model
        self.departure = PerLaneDeparture(departure_rate)

    def run(self):
        with open('data/{}/timing.csv'.format(self.args.folder_prefix), 'w') as file:
            i = 0
            while True:
                queue, arrival = self.input.get()

                t1 = time.time()
                lights = self.model.optimize(queue, arrival, self.departure, verbose=self.args.verbose)
                t2 = time.time()

                file.write('{},{}\n'.format(i, t2 - t1))
                file.flush()

                self.output.put(lights)
                i += 1


class MPCRunner(Runner):
    def __init__(self, args):
        super().__init__(args)

    def run(self):
        options = GurobiOptions(**vars(self.args))
        model = GurobiIntersection(self.configuration_loader.gurobi_config, options)

        with open('data/{}/options.json'.format(self.args.folder_prefix), 'w') as options_file:
            options_file.write(json.dumps(options.__dict__))
            options_file.flush()

        print('Sumo Object created')
        self.sim.start()

        sim_continue_flag = self.sim.step()

        worker_process = Optimizer(model, self.configuration_loader.departure_rate, self.args)
        worker_process.start()

        try:
            while sim_continue_flag:
                queue = self.sim.queue
                print('Queue: {}'.format(queue))

                arr = self.sim.arrival_prediction()

                worker_process.input.put((queue, arr))

                for i in range(20):
                    t1 = time.time()
                    sim_continue_flag = self.sim.step()
                    t2 = time.time()

                    if not sim_continue_flag:
                        return False

                    if not self.args.no_delay and t2 - t1 < 0.05:
                        time.sleep(0.05 - (t2 - t1))

                self.sim.lights = worker_process.output.get()
        finally:
            worker_process.kill()
