class Schedule:
    pass

    # TODO: Implement conflict, green safety, yellow duration and starvation analysis

    def legal(self, history):
        raise NotImplementedError()

    def compatible(self, history, time):
        raise NotImplementedError()

    def empty(self, time):
        raise NotImplementedError()
