import os

from sismic.interpreter import Interpreter
from sismic.io import import_from_yaml

from .models import Schedule


class CoordinationModelImporter:
    def __init__(self):
        self.basepath = os.path.dirname(__file__)

        with open(os.path.join(self.basepath, 'statecharts/coordination.yaml')) as f:
            self.yaml = f.read()

    def construct_statechart(self, signal_id: int, group_id: int, n_groups: int):
        initial_context = {
            'signal_id': signal_id,
            'group_id': group_id,
            'ngi': n_groups,
            'Schedule': Schedule
        }

        statechart = import_from_yaml(self.yaml)
        return Interpreter(statechart, initial_context=initial_context)
