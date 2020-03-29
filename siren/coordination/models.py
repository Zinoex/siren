class Schedule:
    """
    Base schedule. Defaults to saying that the schedule is empty at all time steps and therefore,
    it is both legal and compatible with any history and timestamp.
    Should be inherited for time schedules and temporary schedules (e.g. next 20 seconds).

    All schedules should expect to be preempted if new schedules arrive but the switch will happen at a safe
    timestamp (no change soon in new schedule).
    """

    def legal(self, history):
        return self.conflict_safe() and \
               self.green_interval_safe(history) and \
               self.intermediate_duration_safe(history) and \
               self.legal_phases() and \
               self.starvation_safe(history) and \
               self.minimum_green_safe(history)

    def compatible(self, history, time, old_schedule):
        return True

    def empty(self, time):
        return True

    def conflict_safe(self):
        return True

    def green_interval_safe(self, history):
        return True

    def intermediate_duration_safe(self, history):
        return True

    def legal_phases(self):
        return True

    def starvation_safe(self, history):
        return True

    def minimum_green_safe(self, history):
        return True
