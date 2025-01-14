class EveryNSteps:
    def __init__(self, interval=None):
        self.interval = interval
        self.next_value = interval

    def reset(self):
        self.next_value = self.interval

    def check(self, x):
        if self.interval is None or self.interval < 0:
            return False
        sign = False
        while x >= self.next_value:
            self.next_value += self.interval
            sign = True
        return sign

    def standard(self, x):
        return int(x // self.interval) * self.interval
