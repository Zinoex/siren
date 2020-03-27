class Schedule:
    pass


class Round:
    def __init__(self, n: int, t: float, s: Schedule):
        self.n = n
        self.t = t
        self.s = s