import numpy as np

#creating the bycicle class
class Bicycle():
    def __init__(self):
        self.xc = 0     #pose in x axis
        self.yc = 0     #pose in y axis
        self.theta = 0  #yaw angle
        self.delta = 0  #steering angle
        self.beta = 0   #slip angle
        
        self.L = 2  #distance bet the 2 wheel bases
        self.lr = 1.2   #distace between rear wheel base and centre of gravity
       
        
        self.sample_time = 0.01
        
    def reset(self):
        self.xc = 0
        self.yc = 0
        self.theta = 0
        self.delta = 0
        self.beta = 0
        
        
    #wr and wl are the angular velocities of the left and right rear wheels, delta is the steering angle
    def step(self, wr, wl, delta):
        

        r = np.arctan(delta) / self.L #radius of the curvature - if any
        v = r * (wr + wl) / 2 #speed of the bike
        
        #implementing the differential equations
        xc_dot = v * np.cos(self.theta + self.beta) #rate of change of pose in x axis
        yc_dot = v * np.sin(self.theta + self.beta) #rate of change of y in the y axis
        theta_dot = (v / self.L) * (np.cos(self.beta) * np.tan(self.delta)) #rate of change of yaw
        
        
        self.beta = np.arctan(self.lr * np.tan(self.delta) / self.L) # new slip angle
        
        #update equations using the sampling time
        self.xc += xc_dot * self.sample_time
        self.yc += yc_dot * self.sample_time
        self.theta += theta_dot * self.sample_time 
        
        self.delta = delta

# you can test the model by having an initial delta angle and initial angular velocities for the wheels
# then you loop through over the step functions at the step time increments and calling the step function 
#with the desired parameters


#simple test 
model = Bicycle()
print("initialized bike")
print("parameters before step are all initialized to zero")

for i in range(20):
    model.step(2.8, 2.9, 0.23) 
    #entered the same parameters through all 20 steps ~ constant curvature and speed
    
    print("\npose after single step: \npose in x = ", model.xc, 
          "\npose in y = ", model.yc, "\nyaw angle = ", model.theta)