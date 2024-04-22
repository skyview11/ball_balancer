import time

class LowPassFilter:
    def __init__(self, l):
        self.prev_val = 0
        self.l = l
        self.prev_time = time.time()
    def run(self, value):
        cur_time = time.time()
        ldt = self.l*(cur_time-self.prev_time)
        cur_val = ldt/(1+ldt)*value + 1/(1+ldt)*self.prev_val
        
        self.prev_time = cur_time
        self.prev_val = cur_val
        return cur_val