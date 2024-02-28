import os
import stat
from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from shapely.geometry import *
import math


def wrap(angle, lower_bound, upper_bound):
    """
    Wrap an angle to be within the specified bounds.

    Args:
        angle (float): The angle to be wrapped.
        lower_bound (float): The lower bound for the angle.
        upper_bound (float): The upper bound for the angle.

    Returns:
        float: The wrapped angle.
    """
    range_width = upper_bound - lower_bound
    wrapped_angle = (angle - lower_bound) % range_width + lower_bound

    return wrapped_angle


class Agent():
    def __init__(self, name, x, y, time, theta = None, v = None, omega = None, addnoise = False):
        if theta is None: theta = np.zeros_like(time)
        if v is None: v = np.zeros_like(time)
        if omega is None: omega = np.zeros_like(time)
        
        self.name = name
        # self.x = x
        # self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        
        self.x = x + np.random.normal(0, .035, x.size) if addnoise else x
        self.y = y + np.random.normal(0, .035, y.size) if addnoise else y
        # self.theta = theta + np.random.normal(0, .1, theta.size) if addnoise else theta
        # self.v = v + np.random.normal(0, .005, v.size) if addnoise else v
        # self.omega = omega + np.random.normal(0, .028, omega.size) if addnoise else omega
        self.time = time
        self.gr = False
        self.rotating = False
        
    def goal_reached(self, t, goal):
        # return wrap(math.atan2(goal.p(t).y - self.p(t).y, goal.p(t).x - self.p(t).x), 0, 2*np.pi)
        if goal.p(t) != goal.p(t-1): 
            return 1
        return 0
    
    # def alignment(self, t, goal):
    #     # return wrap(math.atan2(goal.p(t).y - self.p(t).y, goal.p(t).x - self.p(t).x), 0, 2*np.pi)
    #     if goal.p(t) != goal.p(t-1):
    #         self.gr = True 
    #         return 0
    #     elif self.gr and not self.start_rotating and self.omega[t] == 0:
    #         return 1
    #     elif self.gr and not self.start_rotating and self.omega[t] != 0:
    #         self.start_rotating = True
    #         return 1
    #     elif self.gr and self.start_rotating and self.omega[t] != 0:
    #         return 1
    #     elif self.gr and self.start_rotating and self.omega[t] == 0:
    #         self.gr = False 
    #         self.start_rotating = False
    #         return 0
    #     return 0
    
    def alignment(self, t, goal):
        # return wrap(math.atan2(goal.p(t).y - self.p(t).y, goal.p(t).x - self.p(t).x), 0, 2*np.pi)
        if goal.p(t) != goal.p(t-1):
            self.gr = True 
            # return 1
        elif self.gr and not self.rotating and self.omega[t] == 0:
            return 1
        elif self.gr and not self.rotating and self.omega[t] != 0:
            self.rotating = True
            return 1
        elif self.gr and self.rotating and self.v[t] != 0 and self.omega[t] != 0:
            return 1
        elif self.gr and self.rotating and self.v[t] == 0:
            return 1
        elif self.gr and self.rotating and self.v[t] != 0:
            self.gr = False 
            self.rotating = False
            # return 0
        return 0
    
    def task(self, t, goal):
        # return wrap(math.atan2(goal.p(t).y - self.p(t).y, goal.p(t).x - self.p(t).x), 0, 2*np.pi)
        if goal.p(t) != goal.p(t-1):
            self.gr = True 
            # return 1
        elif self.gr and not self.rotating and self.omega[t] == 0:
            return 1
        elif self.gr and not self.rotating and self.omega[t] != 0:
            self.gr = False
            return 0
        return 0
        
    def p(self, t):
        """
        Position

        Args:
            t (int): time index

        Returns:
            Point: position
        """
        return Point(self.x[t], self.y[t])
    
    def dt(self, t):
        return self.time[t]-self.time[t-1]
    
    # def myv(self, t):
    #     return math.sqrt(((self.x[t]-self.x[t-1])/self.dt(t))**2 + ((self.y[t]-self.y[t-1])/self.dt(t))**2)
    
    # def mydv(self, t):
    #     return Point((self.x[t]-self.x[t-1])/self.dt(t), (self.y[t]-self.y[t-1])/self.dt(t))
    
    # def mydist(self, t, obs):
    #     return math.sqrt(((self.x[t-1] + self.dt(t-1)*self.mydv(t-1).x) - obs.x[t-1])**2 +
    #                      ((self.y[t-1] + self.dt(t-1)*self.mydv(t-1).y) - obs.y[t-1])**2)
    
    def dv(self, t):
        """
        decomposed velocity

        Args:
            t (int): time index

        Returns:
            Point: decomposed velocity
        """
        return Point(self.v[t]*math.cos(self.theta[t]), self.v[t]*math.sin(self.theta[t]))
    
    def dist(self, t, obs):
        """
        Distance to obstacle

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: distance
        """
        return self.p(t).distance(obs.p(t))
    
    def heading(self, t, obj):
        """
        heading angle
        
        Args:
            t (int): time index
            obj (Agent): object
            
        Returns:
            float: heading angle
        """
        
        angle = wrap(2*np.pi - wrap(math.atan2(obj.p(t).y - self.p(t).y, obj.p(t).x - self.p(t).x) - wrap(self.theta[t], 0, 2*np.pi), 0, 2*np.pi), -np.pi, np.pi)
        return angle
    
    def risk(self, t, obs):
        """
        Risk

        Args:
            robot (RobotState): robot state
            people (TrackedPersons): tracked people
        """
        
        # risk = 0
        risk = self.v[t] / self.dist(t, obs)
        # risk = self.v[t]/20
        # risk = np.random.normal(0, 0.02)
        collision = False
        
        # Calculate relative velocity vector
        Vrel = Point(obs.dv(t).x - self.dv(t).x, obs.dv(t).y - self.dv(t).y)
            
        # Compute the slope of the line AB and line PAB ⊥ AB
        slope_AB = (obs.p(t).y - self.p(t).y) / (obs.p(t).x - self.p(t).x) 
        slope_PAB = -1 / slope_AB
        # Choose distances to the left and right of point B

        # Calculate coordinates for two points along the perpendicular line
        # Calculate the change in x and y based on the fixed horizontal distance
        delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
        delta_y = delta_x * slope_PAB
        # Calculate coordinates for two points along the perpendicular line
        left = Point(obs.p(t).x - delta_x, obs.p(t).y - delta_y)
        right = Point(obs.p(t).x + delta_x, obs.p(t).y + delta_y)
        # Cone
        cone_origin = Point(self.p(t).x, self.p(t).y)               
        cone = Polygon([cone_origin, left, right])
        
        P = Point(cone_origin.x + self.dv(t).x, cone_origin.y + self.dv(t).y)
        # collision = P.within(cone) and dobs < SAFE_DIST
        collision = P.within(cone) and self.dist(t, obs) < SAFE_DIST
        if collision:
            time_collision_measure = self.dist(t, obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
            steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
            # risk = risk + 1/time_collision_measure + steering_effort_measure
            risk = risk + self.v[t] + 1/time_collision_measure + steering_effort_measure
                    
        return math.exp(risk)
       

if __name__ == '__main__': 
    DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/data'
    PP_DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/ppdata'
    # CSV_NAME = ["data_20240225_221217", "data_20240225_221447"]
    # CSV_NAME = ["data_20240131_234259", "data_20240131_234529", "data_20240225_230735", "data_20240225_231005"]
    CSV_NAME = ["data_20240228_160625"]
    
    dfs = list()
    for CSV in CSV_NAME:
        INPUT_CSV = DATA_DIR + '/' + CSV + '.csv'
        OUTPUT_CSV = PP_DATA_DIR + '/' + CSV + '.csv'
        
        OBS_SIZE = 2.0
        SAFE_DIST = 5.0
        SEL_ID = "1000_"

        # Read the CSV into a pandas DataFrame
        data = pd.read_csv(INPUT_CSV)
        noise_sz = data["r_x"].size
        R = Agent("R", data["r_x"], data["r_y"], data["time"], data["r_{\theta}"], data["r_v"], data["r_{\omega}"], addnoise = True)
        H = Agent("H", data["h_" + SEL_ID + "x"], data["h_" + SEL_ID + "y"], data["time"], data["h_" + SEL_ID + "{\theta}"], data["h_" + SEL_ID + "v"], data["h_" + SEL_ID + "{\omega}"], addnoise = True)
        RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], data["time"])
        HG = Agent("HG", data["h_" + SEL_ID + "{gx}"], data["h_" + SEL_ID + "{gy}"], data["time"])
                
        df = pd.DataFrame(columns=["t", "g_r", "v", r"\theta_{g}", "d_g", "col", "r", r"\omega", r"d_{obs}"])  
        
        I0 = 2
        for i in range(I0, len(data)-1):
            df.loc[i] = {
                        "t": H.task(i+1, HG) + np.random.normal(0, 0.005),
                        # "t": H.alignment(i+1, HG),
                        "g_r": H.goal_reached(i+1, HG),
                        # "g_r": H.goal_reached(i+1, HG)+np.random.uniform(-0.0075, 0.0075), # FIXME: ALMOST
                        "v" : H.v[i],
                        "d_g" : H.dist(i, HG),
                        r"\theta_{g}" : H.heading(i, HG),
                        "r" : H.risk(i-1, R), 
                        r"\omega" : H.omega[i],
                        r"d_{obs}" : H.dist(i, R) + np.random.uniform(-0.1, 0.1),
                        }

        # Save the processed data to another CSV file
        df = df[I0:-1]
        df.to_csv(OUTPUT_CSV, index = False)
        dfs.append(df)
        
    fdf = pd.concat(dfs, ignore_index=True, sort=False)
    columns=["t", "g_r", "v", r"\theta_{g}", "d_g", "r", r"\omega", r"d_{obs}"]
    fdf[columns].plot()
    legend = ['$' + k + '$' for k in columns]
    plt.legend(legend)
    plt.show()