
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
        self.vehicle_rate = 100
        self.v = 17
        self.speed_variance = 0
        self.self_driving_vehicle_proportion = 0 #number between 0 and 1, 0 means no self driving vehicles, 1 means entirely self driving vehicles
        if self.self_driving_vehicle_proportion == 1:
            self.v = self.v * 1.5
#----------------------------------------------------------------------------------------------------------------------------------------------------#
    #this section defines all the paths that a vehicle can take
    # SOUTH, EAST, NORTH, WEST
        #INNER, OUTER
        # Intersection in
            #paths 0-7

        # South In (Inner)
        self.sim.create_segment(
            (lane_space/2 + island_width/2, length + intersection_size/2),
            (lane_space/2 + island_width/2, intersection_size/2))
        # South In (Outer)
        self.sim.create_segment(
            (lane_space*3/2 + island_width/2, length+intersection_size/2),
            (lane_space*3/2+island_width/2, intersection_size/2))

        # East In (Inner)
        self.sim.create_segment(
            (length + intersection_size/2, -lane_space/2 - island_width/2),
            (intersection_size/2, -lane_space/2 - island_width/2))
        # East In (Outer)
        self.sim.create_segment(
            (length + intersection_size/2, -lane_space*3/2 - island_width/2),
            (intersection_size/2, - lane_space*3/2 - island_width/2))

        # North In (Inner)
        self.sim.create_segment(
            (-lane_space/2 - island_width/2, -length - intersection_size/2),
            (-lane_space/2 - island_width/2, - intersection_size/2))
        # North In (Outer)
        self.sim.create_segment(
            (-lane_space*3/2 - island_width/2, -length - intersection_size/2),
            (-lane_space*3/2 - island_width/2, -intersection_size/2))

        # West In (Inner)
        self.sim.create_segment(
            (-length - intersection_size/2, lane_space/2 + island_width/2),
            (-intersection_size/2, lane_space/2 + island_width/2))
        # West In (Outer)
        self.sim.create_segment(
            (-length - intersection_size/2, lane_space*3/2 + island_width/2),
            (-intersection_size/2, lane_space*3/2 + island_width/2))

        # Intersection out
            #paths 8-15

        # South Out (Inner)
        self.sim.create_segment(
            (-lane_space/2 - island_width/2, intersection_size/2),
            (-lane_space/2 - island_width/2, length + intersection_size/2))
        # South Out (Outer)
        self.sim.create_segment(
            (-lane_space*3/2 - island_width/2, intersection_size/2),
            (-lane_space*3/2 - island_width/2, length + intersection_size/2))

        # East Out (Inner)
        self.sim.create_segment(
            (intersection_size/2, lane_space/2 + island_width/2),
            (length+intersection_size/2, lane_space/2 + island_width/2))
        # East Out (Outer)
        self.sim.create_segment(
            (intersection_size/2, lane_space*3/2 + island_width/2),
            (length+intersection_size/2, lane_space*3/2 + island_width/2))

        # North Out (Inner)
        self.sim.create_segment(
            (lane_space/2 + island_width/2, -intersection_size/2),
            (lane_space/2 + island_width/2, -length - intersection_size/2))
        # North Out (Outer)
        self.sim.create_segment(
            (lane_space*3/2 + island_width/2, -intersection_size/2),
            (lane_space*3/2 + island_width/2, -length-intersection_size/2))

        # West Out (Inner)
        self.sim.create_segment(
            (-intersection_size/2, -lane_space/2 - island_width/2),
            (-length-intersection_size/2, -lane_space/2 - island_width/2))
        # West Out (Outer)
        self.sim.create_segment(
            (-intersection_size/2, -lane_space*3/2 - island_width/2),
            (-length - intersection_size/2, -lane_space*3/2 - island_width/2))

        # Straight
            #paths 16-23

        # South to North (Inner)
        self.sim.create_segment(
            (lane_space/2 + island_width/2, intersection_size/2),
            (lane_space/2 + island_width/2, -intersection_size/2))
        # South to North (Outer)
        self.sim.create_segment(
            (lane_space*3/2 + island_width/2, intersection_size/2),
            (lane_space*3/2 + island_width/2, -intersection_size/2))

        # East to West (Inner)
        self.sim.create_segment(
            (intersection_size/2, -lane_space/2 - island_width/2),
            (-intersection_size/2, -lane_space/2 - island_width/2))
        # East to West (Outer)
        self.sim.create_segment(
            (intersection_size/2, -lane_space*3/2 - island_width/2),
            (-intersection_size/2, -lane_space*3/2 - island_width/2))

        # North to South (Inner)
        self.sim.create_segment(
            (-lane_space/2 - island_width/2, -intersection_size/2),
            (-lane_space/2 - island_width/2, intersection_size/2))
        # North to South (Outer)
        self.sim.create_segment(
            (-lane_space*3/2 - island_width/2, -intersection_size/2),
            (-lane_space*3/2 - island_width/2, intersection_size/2))

        # West to East (Inner)
        self.sim.create_segment(
            (-intersection_size/2, lane_space/2 + island_width/2),
            (intersection_size/2, lane_space/2 + island_width/2))
        # West to East (Outer)
        self.sim.create_segment(
            (-intersection_size/2, lane_space*3/2 + island_width/2),
            (intersection_size/2, lane_space*3/2 + island_width/2))
    
    # SOUTH, EAST, NORTH, WEST
        #Right turn
            #paths 24-27

        # South to West? (This is a left turn not a right turn)
        self.sim.create_quadratic_bezier_curve(
            (lane_space*3/2 + island_width/2, intersection_size/2),
            (lane_space*3/2 + island_width/2, lane_space*3/2 + island_width/2),
            (intersection_size/2, lane_space*3/2 + island_width/2))
        # East to North
        self.sim.create_quadratic_bezier_curve(
            (intersection_size/2, -lane_space*3/2 - island_width/2),
            (lane_space*3/2 + island_width/2, -lane_space*3/2 - island_width/2),
            (lane_space*3/2 + island_width/2, -intersection_size/2))
        # North to East? (This is a left turn not a right turn)
        self.sim.create_quadratic_bezier_curve(
            (-lane_space*3/2 - island_width/2, -intersection_size/2),
            (-lane_space*3/2 - island_width/2, -lane_space*3/2 - island_width/2),
            (-intersection_size/2, -lane_space*3/2 - island_width/2))
        # West to South
        self.sim.create_quadratic_bezier_curve(
            (-intersection_size/2, lane_space*3/2 + island_width/2),
            (-lane_space*3/2 - island_width/2, lane_space*3/2 + island_width/2),
            (-lane_space*3/2 - island_width/2, intersection_size/2))

        # Left turn
            #paths 28-31

        # South to East? (This is a right turn not a left turn)
        self.sim.create_quadratic_bezier_curve(
            (lane_space/2 + island_width/2, intersection_size/2),
            (lane_space/2 + island_width/2, -lane_space/2 - island_width/2),
            (-intersection_size/2, -lane_space/2 - island_width/2))
        # East to South
        self.sim.create_quadratic_bezier_curve(
            (intersection_size/2, -lane_space/2 - island_width/2),
            (-lane_space/2 - island_width/2, -lane_space/2 - island_width/2),
            (-lane_space/2 - island_width/2, intersection_size/2))
        # North to West? (This is a right turn not a left turn)
        self.sim.create_quadratic_bezier_curve(
            (-lane_space/2 - island_width/2, -intersection_size/2),
            (-lane_space/2 - island_width/2, lane_space/2 + island_width/2),
            (intersection_size/2, lane_space/2 + island_width/2))
        # West to North
        self.sim.create_quadratic_bezier_curve(
            (-intersection_size/2, lane_space/2 + island_width/2),
            (lane_space/2 + island_width/2, lane_space/2 + island_width/2),
            (lane_space/2 + island_width/2, -intersection_size/2))

        #Emergency Vehicvle Lanes

        # South In (Left Turn)
        self.sim.create_segment(
            (lane_space*5/2 + island_width/2 + island_width, length + intersection_size/2), 
            (lane_space*5/2 + island_width/2 + island_width, intersection_size))
        # South In (Straight)
        self.sim.create_segment(
            (lane_space*7/2 + island_width/2 + island_width, length + intersection_size/2), 
            (lane_space*7/2 + island_width/2 + island_width, intersection_size))
        # South In (Right Turn)
        self.sim.create_segment(
            (lane_space*9/2 + island_width/2 + island_width, length + intersection_size/2), 
            (lane_space*9/2 + island_width/2 + island_width, intersection_size))
        # East Out (Left Turn)
        self.sim.create_segment(
            (length + intersection_size/2, lane_space*5/2 + island_width/2 + island_width), 
            (intersection_size, lane_space*5/2 + island_width/2 + island_width))
        # East Out (Straight)
        self.sim.create_segment(
            (length + intersection_size/2, lane_space*7/2 + island_width/2 + island_width), 
            (intersection_size, lane_space*7/2 + island_width/2 + island_width))
        # East Out (Right Turn)
        self.sim.create_segment(
            (length + intersection_size/2, lane_space*9/2 + island_width/2 + island_width), 
            (intersection_size, lane_space*9/2 + island_width/2 + island_width))
        #Curve South to East
        self.sim.create_quadratic_bezier_curve(
            (lane_space*9/2 + island_width/2 + island_width, intersection_size), 
            (lane_space*9/2 + island_width/2 + island_width, lane_space*9/2 + island_width/2 + island_width), 
            (intersection_size, lane_space*9/2 + island_width/2 + island_width))

        # East In (Left Turn)
        self.sim.create_segment(
            (length + intersection_size/2, -lane_space*5/2 - island_width/2 - island_width), 
            (intersection_size, -lane_space*5/2 - island_width/2 - island_width))
        # East In (Straight)
        self.sim.create_segment(
            (length + intersection_size/2, -lane_space*7/2 - island_width/2 - island_width), 
            (intersection_size, -lane_space*7/2 - island_width/2 - island_width))
        # East In (Right Turn)
        self.sim.create_segment(
            (length + intersection_size/2, -lane_space*9/2 - island_width/2 - island_width), 
            (intersection_size, -lane_space*9/2 - island_width/2 - island_width))
        # North Out (Left Turn)
        self.sim.create_segment(
            (lane_space*5/2 + island_width/2 + island_width, -length - intersection_size/2), 
            (lane_space*5/2 + island_width/2 + island_width, -intersection_size))
        #North Out (Straight)
        self.sim.create_segment(
            (lane_space*7/2 + island_width/2 + island_width, -length - intersection_size/2), 
            (lane_space*7/2 + island_width/2 + island_width, -intersection_size))
        # North Out (Right Turn)
        self.sim.create_segment(
            (lane_space*9/2 + island_width/2 + island_width, -length - intersection_size/2), 
            (lane_space*9/2 + island_width/2 + island_width, -intersection_size))
        #Curve East to North
        self.sim.create_quadratic_bezier_curve(
            (intersection_size, -lane_space*9/2 - island_width/2 - island_width), 
            (lane_space*9/2 + island_width/2 + island_width, -lane_space*9/2 - island_width/2 - island_width), 
            (lane_space*9/2 + island_width/2 + island_width, -intersection_size))

        # North In (Left Turn)
        self.sim.create_segment(
            (-lane_space*5/2 - island_width/2 - island_width, -length - intersection_size/2), 
            (-lane_space*5/2 - island_width/2 - island_width, -intersection_size))
        #North In (Straight)
        self.sim.create_segment(
            (-lane_space*7/2 - island_width/2 - island_width, -length - intersection_size/2), 
            (-lane_space*7/2 - island_width/2 - island_width, -intersection_size))
        # North In (Right Turn)
        self.sim.create_segment(
            (-lane_space*9/2 - island_width/2 - island_width, -length - intersection_size/2), 
            (-lane_space*9/2 - island_width/2 - island_width, -intersection_size))
        # West Out (Left Turn)
        self.sim.create_segment(
            (-length - intersection_size/2, -lane_space*5/2 - island_width/2 - island_width), 
            (-intersection_size, -lane_space*5/2 - island_width/2 - island_width))
        #West Out (Straight)
        self.sim.create_segment(
            (-length - intersection_size/2, -lane_space*7/2 - island_width/2 - island_width), 
            (-intersection_size, -lane_space*7/2 - island_width/2 - island_width))
        # West Out (Right Turn)
        self.sim.create_segment(
            (-length - intersection_size/2, -lane_space*9/2 - island_width/2 - island_width), 
            (-intersection_size, -lane_space*9/2 - island_width/2 - island_width))
        #Curve North to West
        self.sim.create_quadratic_bezier_curve(
            (-lane_space*9/2 - island_width/2 - island_width, -intersection_size), 
            (-lane_space*9/2 - island_width/2 - island_width, -lane_space*9/2 - island_width/2 - island_width), 
            (-intersection_size, -lane_space*9/2 - island_width/2 - island_width))

        # West In (Left Turn)
        self.sim.create_segment(
            (-length - intersection_size/2, lane_space*5/2 + island_width/2 + island_width), 
            (-intersection_size, lane_space*5/2 + island_width/2 + island_width))
        #West In (Straight)
        self.sim.create_segment(
            (-length - intersection_size/2, lane_space*7/2 + island_width/2 + island_width), 
            (-intersection_size, lane_space*7/2 + island_width/2 + island_width))
        # West In (Right Turn)
        self.sim.create_segment(
            (-length - intersection_size/2, lane_space*9/2 + island_width/2 + island_width), 
            (-intersection_size, lane_space*9/2 + island_width/2 + island_width))
        # South Out (Left Turn)
        self.sim.create_segment(
            (-lane_space*5/2 - island_width/2 - island_width, intersection_size), 
            (-lane_space*5/2 - island_width/2 - island_width, length + intersection_size/2))
        #South Out (Straight)
        self.sim.create_segment(
            (-lane_space*7/2 - island_width/2 - island_width, intersection_size), 
            (-lane_space*7/2 - island_width/2 - island_width, length + intersection_size/2))
        # South Out (Right Turn)
        self.sim.create_segment(
            (-lane_space*9/2 - island_width/2 - island_width, intersection_size), 
            (-lane_space*9/2 - island_width/2 - island_width, length + intersection_size/2))
        #Curve West to South
        self.sim.create_quadratic_bezier_curve(
            (-intersection_size, lane_space*9/2 + island_width/2 + island_width), 
            (-lane_space*9/2 - island_width/2 - island_width, lane_space*9/2 + island_width/2 + island_width), 
            (-lane_space*9/2 - island_width/2 - island_width, intersection_size))

        #Emergency Vehicle Straight Lanes
        #South to North
        self.sim.create_segment(
            (lane_space*7/2 + island_width/2 + island_width, intersection_size), 
            (lane_space*7/2 + island_width/2 + island_width, -intersection_size), color=(100, 100, 225))
        #East to West
        self.sim.create_segment(
            (intersection_size, lane_space*7/2 + island_width/2 + island_width), 
            (-intersection_size, lane_space*7/2 + island_width/2 + island_width), color=(100, 100, 225))
        #North to South
        self.sim.create_segment(
            (-lane_space*7/2 - island_width/2 - island_width, -intersection_size), 
            (-lane_space*7/2 - island_width/2 - island_width, intersection_size), color=(100, 100, 225))
        #West to East
        self.sim.create_segment(
            (-intersection_size, -lane_space*7/2 - island_width/2 - island_width), 
            (intersection_size, -lane_space*7/2 - island_width/2 - island_width), color=(100, 100, 225))
        
        #Emergency Vehicle Left Turns

        #South to West
        self.sim.create_quadratic_bezier_curve(
            (lane_space*5/2 + island_width/2 + island_width, intersection_size), 
            (lane_space*5/2 + island_width/2 + island_width, -lane_space*5/2 - island_width/2 - island_width), 
            (-intersection_size, -lane_space*5/2 - island_width/2 - island_width), 
            color=(100, 100, 225))
        
        #North to East
        self.sim.create_quadratic_bezier_curve(
            (-lane_space*5/2 - island_width/2 - island_width, -intersection_size), 
            (-lane_space*5/2 - island_width/2 - island_width, lane_space*5/2 + island_width/2 + island_width), 
            (intersection_size, lane_space*5/2 + island_width/2 + island_width), 
            color=(100, 100, 225))
        
        #East to South
        self.sim.create_quadratic_bezier_curve(
            (intersection_size, -lane_space*5/2 - island_width/2 - island_width), 
            (-lane_space*5/2 - island_width/2 - island_width, -lane_space*5/2 - island_width/2 - island_width), 
            (-lane_space*5/2 - island_width/2 - island_width, intersection_size), 
            color=(100, 100, 225))
        
        #West to North
        self.sim.create_quadratic_bezier_curve(
            (-intersection_size, lane_space*5/2 + island_width/2 + island_width), 
            (lane_space*5/2 + island_width/2 + island_width, lane_space*5/2 + island_width/2 + island_width), 
            (lane_space*5/2 + island_width/2 + island_width, -intersection_size), 
            color=(100, 100, 225))
        
    #all interfearing paths
    
        #left turn from the South intersects with the inner and outer straights coming from the North
        self.sim.define_interfearing_paths([0, 28], [4, 20],turn=True)
        self.sim.define_interfearing_paths([0, 28], [5, 21],turn=True)
        #left turn from the Rast intersects with the inner and outer straights coming from the Weat
        self.sim.define_interfearing_paths([2, 29], [6, 22],turn=True)
        self.sim.define_interfearing_paths([2, 29], [7, 23],turn=True)
        #left turn from the North intersects with the inner and outer straights coming from the south
        self.sim.define_interfearing_paths([4, 30], [0, 16],turn=True)
        self.sim.define_interfearing_paths([4, 30], [1, 17],turn=True)
        #left turn from the West intersects with the inner and outer straights coming from the East
        self.sim.define_interfearing_paths([6, 31], [2, 18],turn=True)
        self.sim.define_interfearing_paths([6, 31], [3, 19],turn=True)

        '''
        this section creates vehicle generators, we have two vehicle generators one; that creates regular vehicles (self.vg)
        and one that creates self-driving vehicles (self.sdvg)
        '''
        #regular vehicle generator
        self.vg = VehicleGenerator({
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at

            'vehicles': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [0, 16, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [1, 17, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [1, 24, 11], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [0, 28, 14], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [2, 18, 14], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [3, 19, 15], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [3, 25, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [2, 29, 8], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [4, 20, 8], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [5, 21, 9], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [5, 26, 15], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [4, 30, 10], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [6, 22, 10], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 23, 11], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 27, 9], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [6, 31, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                ], 'vehicle_rate' : self.vehicle_rate*(1-self.self_driving_vehicle_proportion) 
            })
        
        
        #self-driving vehicle generator
        self.sdvg = VehicleGenerator({
 
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at
            #'T' defines the raction time of the vehicle, the base is 1
            #'s0' defines the shortest distance a vehicle is able to drive behind another vehicle
            'vehicles': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [0, 16, 12], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [1, 17, 13], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [1, 24, 11], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [0, 28, 14], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [2, 18, 14], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [3, 19, 15], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [3, 25, 13], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [2, 29, 8], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [4, 20, 8], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [5, 21, 9], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [5, 26, 15], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [4, 30, 10], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [6, 22, 10], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [7, 23, 11], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [7, 27, 9], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [6, 31, 12], 'v_max': self.v ,'T' : 0.1,'s0' : 4}),
                ], 'vehicle_rate' : self.vehicle_rate*self.self_driving_vehicle_proportion 
            })
        
        #adding both vehicle generators
        self.sim.add_vehicle_generator(self.vg)
        self.sim.add_vehicle_generator(self.sdvg)

    

        #adding the traffic signal
        self.sim.create_traffic_signal([self.sim.segments[0],self.sim.segments[1], self.sim.segments[4], self.sim.segments[5]],[self.sim.segments[2],self.sim.segments[3],self.sim.segments[6], self.sim.segments[7]])

    
    #this function returns an instance of the simulation defined above
    def get_sim(self):
        return self.sim