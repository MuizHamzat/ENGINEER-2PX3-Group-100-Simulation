from trafficSimulator import *
import numpy as np


class Intersection:
    def __init__(self):
        self.sim = Simulation()
        lane_space = 3.5
        intersection_size = 24
        island_width = 2
        length = 100


#---------------------------------------------------------------Variables----------------------------------------------------------------------------#
        self.vehicle_rate = 10
        self.v = 17
        self.speed_variance = 0
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#
    
        self.sim.create_segment((1.75, 112), (1.75, 12)) #South entrance
        
        self.sim.create_segment((1.75, 12), (1.75, -12))

        self.sim.create_quadratic_bezier_curve((1.75, 12), (1.75, 1.75), (12, 1.75)) #Right turn

        self.sim.create_quadratic_bezier_curve((1.75, 12), (-1.75, -1.75), (-12, -1.75)) #Right turn
    
        self.vg = VehicleGenerator({
            'vehicles': [
                (1,{'path':[0,1], 'v_max': self.v+2*self.speed_variance*np.random.random()-self.speed_variance}),
            ], 'vehicle_rate': self.vehicle_rate*(1-self.self_driving_vehicle_proportion)
        })
        self.sim.add_vehicle_generator(self.vg)
    def get_sim(self):
        return self.sim