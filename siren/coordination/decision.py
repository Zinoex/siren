from typing import Dict, Set

from .models import Round, Schedule


def schedule_decision(sync_dict: Dict[int, Round], active_signals: Set[int]) -> Schedule:
    raise NotImplementedError()
