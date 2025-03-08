
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
    #this section defines all the paths that a vehicle can take
    # SOUTH, EAST, NORTH, WEST
        #INNER, OUTER

        # region Emergency Vehicle Left Turns (paths 0-3)
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
        # endregion

# ORIGINAL INTERSECTION~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        # region Intersection in (paths 4-11)

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
        # endregion

        # region Intersection out (paths 12-19)

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
        # endregion

        # region Straight Paths (paths 20-27)

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
        # endregion
    
    # SOUTH, EAST, NORTH, WEST
        # region Right turn paths (paths 28-31)

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
        # endregion

        # region Left turn paths (paths 32-35)

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
        # endregion

#~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~#

        #Emergency Vehicvle Lanes

        #region South In East Out Lanes (paths 36-42)
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
        #endregion

        #region East In North Out Lanes (paths 43-49)
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
        #endregion

        #region North In West Out Lanes (paths 50-56)
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
        #endregion

        #region West In South Out Lanes (paths 57-63)
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
        #endregion
        

        # region Emergency Vehicle Straight Lanes (paths 64-67)
        #South to North
        self.sim.create_segment(
            (lane_space*7/2 + island_width/2 + island_width, intersection_size), 
            (lane_space*7/2 + island_width/2 + island_width, -intersection_size), color=(173, 216, 230))
        #East to West
        self.sim.create_segment(
            (intersection_size, lane_space*7/2 + island_width/2 + island_width), 
            (-intersection_size, lane_space*7/2 + island_width/2 + island_width), color=(173, 216, 230))
        #North to South
        self.sim.create_segment(
            (-lane_space*7/2 - island_width/2 - island_width, -intersection_size), 
            (-lane_space*7/2 - island_width/2 - island_width, intersection_size), color=(173, 216, 230))
        #West to East
        self.sim.create_segment(
            (-intersection_size, -lane_space*7/2 - island_width/2 - island_width), 
            (intersection_size, -lane_space*7/2 - island_width/2 - island_width), color=(173, 216, 230))
        # endregion
        
        
        # region Pedestrian Bridges
        #South to North
        self.sim.create_segment(
            (lane_space*15/2 + island_width/2 + island_width, intersection_size + 10), 
            (lane_space*15/2 + island_width/2 + island_width, -intersection_size - 10),
              color=(233, 116, 81), width=2.5)
        #East to West
        self.sim.create_segment(
            (intersection_size + 10, lane_space*15/2 + island_width/2 + island_width), 
            (-intersection_size - 10, lane_space*15/2 + island_width/2 + island_width),
              color=(233, 116, 81), width=2.5)
        #North to South
        self.sim.create_segment(
            (-lane_space*15/2 - island_width/2 - island_width, -intersection_size - 10), 
            (-lane_space*15/2 - island_width/2 - island_width, intersection_size + 10),
              color=(233, 116, 81), width=2.5)
        #West to East
        self.sim.create_segment(
            (-intersection_size - 10, -lane_space*15/2 - island_width/2 - island_width), 
            (intersection_size + 10, -lane_space*15/2 - island_width/2 - island_width),
              color=(233, 116, 81), width=2.5)
        # endregion
        
    #region Interfearing paths
    
        #left turn from the South intersects with the inner and outer straights coming from the North
        self.sim.define_interfearing_paths([4, 32], [8, 24],turn=True)
        self.sim.define_interfearing_paths([4, 32], [9, 25],turn=True)
        #left turn from the Rast intersects with the inner and outer straights coming from the Weat
        self.sim.define_interfearing_paths([6, 33], [10, 26],turn=True)
        self.sim.define_interfearing_paths([6, 33], [11, 27],turn=True)
        #left turn from the North intersects with the inner and outer straights coming from the south
        self.sim.define_interfearing_paths([8, 34], [4, 20],turn=True)
        self.sim.define_interfearing_paths([8, 34], [5, 21],turn=True)
        #left turn from the West intersects with the inner and outer straights coming from the East
        self.sim.define_interfearing_paths([10, 35], [6, 22],turn=True)
        self.sim.define_interfearing_paths([10, 35], [7, 23],turn=True)
    #endregion

        '''
        this section creates vehicle generators, we have two vehicle generators one; that creates regular vehicles (self.vg)
        and one that creates self-driving vehicles (self.sdvg)
        '''
        # region Regular vehicle generator
        self.vg = VehicleGenerator({
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at

            'vehicles': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [4, 20, 16], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [5, 21, 17], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [5, 28, 15], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [4, 32, 18], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [6, 22, 18], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 23, 19], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [7, 29, 17], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [6, 33, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [8, 24, 12], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [9, 25, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [9, 30, 19], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [8, 34, 14], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [10, 26, 14], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [11, 27, 15], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [11, 31, 13], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                (1, {'path': [10, 35, 16], 'v_max': self.v+ 2*self.speed_variance*np.random.random() -self.speed_variance}),
                ], 'vehicle_rate' : self.vehicle_rate*(1-self.self_driving_vehicle_proportion) 
            })
        # endregion
        
        
        # region Self-driving vehicle generator
        self.sdvg = VehicleGenerator({
 
            #The first variable: 1 defines the weight if the vehicle; the higher the weight the more likely that type of vehicle will generate
            # 'path' defines the order of segments the vehicle will drive over
            #'v_max' defines the fastest speed a vehicle can drive at
            #'T' defines the raction time of the vehicle, the base is 1
            #'s0' defines the shortest distance a vehicle is able to drive behind another vehicle
            'vehicles': [
                #South [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [4, 20, 16], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [5, 21, 17], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [5, 28, 15], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [4, 32, 18], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),

                #East [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [6, 22, 18], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [7, 23, 18], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [7, 29, 17], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [6, 33, 12], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),

                #North [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [8, 24, 12], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [9, 25, 13], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [9, 30, 19], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [8, 34, 14], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
           
                #West [Inner Straight, Outer Straight, Right Turn, Left Turn]
                (1, {'path': [10, 26, 14], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [11, 27, 15], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [11, 31, 13], 'v_max': self.v, 'T' : 0.1,'s0' : 4}),
                (1, {'path': [10, 35, 16], 'v_max': self.v ,'T' : 0.1,'s0' : 4}),
                ], 'vehicle_rate' : self.vehicle_rate*self.self_driving_vehicle_proportion 
            })
        # endregion
        
        #adding both vehicle generators
        self.sim.add_vehicle_generator(self.vg)
        self.sim.add_vehicle_generator(self.sdvg)

    

        #adding the traffic signal
        self.sim.create_traffic_signal( [self.sim.segments[4], self.sim.segments[5],
                                        self.sim.segments[8], self.sim.segments[9]],
                                        [self.sim.segments[6],self.sim.segments[7],
                                         self.sim.segments[10], self.sim.segments[11]])

    
    #this function returns an instance of the simulation defined above
    def get_sim(self):
        return self.sim