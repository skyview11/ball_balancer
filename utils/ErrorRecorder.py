from .LowPassFilter import LowPassFilter
import numpy as np
import time
class ErrorRecoder:
    def __init__(self, n):
        self.past_errors: np.ndarray = np.zeros((n, 3)) # (x, y, time)
        self.cur_buff_idx = 0
        self.n = n
        self.low_pass_filter = LowPassFilter(50)
        self.inte = np.zeros((2, ))
        self.i_reset = 0
    def addPastError(self, past_err:tuple, time:float):
        self.past_errors[self.cur_buff_idx] = past_err + (time,)
        self.cur_buff_idx = (self.cur_buff_idx + 1) % self.n
        
    def addEmptyStack(self):
        self.past_errors[self.cur_buff_idx] = (0, 0, 0)
        self.cur_buff_idx = (self.cur_buff_idx + 1) % self.n
        
        
    def getVelocity(self, cur_err_vec: tuple):
        time_interval = time.time() - self.past_errors[self.cur_buff_idx-1,2]
        
        vel_vec = [-(cur_err_vec[0]-self.past_errors[self.cur_buff_idx-1,0])/time_interval, 
                   -(cur_err_vec[1]-self.past_errors[self.cur_buff_idx-1,1])/time_interval]
        
        self.addPastError(tuple(cur_err_vec), time.time())
        vel_vec = self.low_pass_filter.run(value=np.array(vel_vec))
        return vel_vec
    def getInte(self, cur_err_vec: tuple):
        time_interval = time.time() - self.past_errors[self.cur_buff_idx-1,2]
        self.inte += time_interval * np.array(cur_err_vec)
        if (time.time() - self.i_reset):
            self.i_reset = time.time()
            self.inte = (0, 0)
        return self.inte