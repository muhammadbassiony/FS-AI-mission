
#simple steering wheel pid controller
class PID():
    def __init__(self, kp, ki, kd):
        self.kp = kp    #control gain
        self.ki = ki    #integral coefficient
        self.kd = kd    #derivative coefficient
        
        self.p_error = 0.0
        self.d_error = 0.0
        self.i_error = 0.0
        
        self.counter = 0
        self.sumerror = 0.0
        self.maxerror = -1
        self.minerror = 999999
        
        # previous Cross Track Error - cte
        self.prev_cte = 0.0;
        
    def updateError(self, cte):
        self.p_error = cte
        self.i_error += cte
        
        self.d_error = cte - self.prev_cte
        
        self.sum += cte
        self.counter += 1
        
        if(cte > self.maxError):
            self.maxerror = cte
        
        if(cte < self.minerror):
            self.minerror = cte
            
            
    def getTotalError(self):
        return ((self.p_error * self.Kp) + (self.i_error * self.Ki) + (self.d_error * self.Kd))
        

  