class Schedule:
    pass


class Round:
    def __init__(self, n: int, s: Schedule):
        self.n = n  # Round index
        self.s = s  # Schedule
