from typing import Dict, Union, Set
import statistics

from .models import Round, Schedule


def time_decision(sync_dict: Dict[Union[int, str], Round], active_signals: Set[Union[int, str]]) -> float:
    timestamps = [sync_round.t for signal, sync_round in sync_dict if signal in active_signals]
    return statistics.mean(timestamps)


def schedule_decision(sync_dict: Dict[Union[int, str], Round], active_signals: Set[Union[int, str]]) -> Schedule:
    raise NotImplementedError()