import os

from sismic.interpreter import Interpreter
from sismic.io import import_from_yaml

from .models import Round
from .decision import time_decision, schedule_decision


class CoordinationModelImporter:
    def __init__(self):
        self.basepath = os.path.dirname(__file__)

        with open(os.path.join(self.basepath, 'statecharts/coordination.yaml')) as f:
            self.yaml = f.read()

    def construct_statechart(self, signal_id, group_id, n_groups):
        initial_context = {
            'signal_id': signal_id,
            'group_id': group_id,
            'ngi': n_groups,
            'time_decision': time_decision,
            'schedule_decision': schedule_decision,
            'Round': Round
        }

        statechart = import_from_yaml(self.yaml)
        return Interpreter(statechart, initial_context=initial_context)
