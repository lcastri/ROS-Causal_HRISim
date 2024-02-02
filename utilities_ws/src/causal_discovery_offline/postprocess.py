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
    def __init__(self, name, x, y, theta, v, omega, time) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        self.time = time
        
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
    
    def myv(self, t):
        return math.sqrt(((self.x[t]-self.x[t-1])/self.dt(t))**2 + ((self.y[t]-self.y[t-1])/self.dt(t))**2)
    
    def mydv(self, t):
        return Point((self.x[t]-self.x[t-1])/self.dt(t), (self.y[t]-self.y[t-1])/self.dt(t))
    
    def mydist(self, t, obs):
        return math.sqrt(((self.x[t-1] + self.dt(t-1)*self.mydv(t-1).x) - obs.x[t-1])**2 +
                         ((self.y[t-1] + self.dt(t-1)*self.mydv(t-1).y) - obs.y[t-1])**2)
    
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
    
    
    # def risk(self, t, obs):
    #     """
    #     Risk

    #     Args:
    #         robot (RobotState): robot state
    #         people (TrackedPersons): tracked people
    #     """
        
    #     risk = self.myv(t)
    #     collision = False
        
    #     # Calculate relative velocity vector
    #     Vrel = Point(obs.mydv(t).x - self.mydv(t).x, obs.mydv(t).y - self.mydv(t).y)
            
    #     # Compute the slope of the line AB and line PAB ⊥ AB
    #     slope_AB = (obs.p(t).y - self.p(t).y) / (obs.p(t).x - self.p(t).x)  # Rise over run
    #     slope_PAB = -1 / slope_AB
    #     # Choose distances to the left and right of point B

    #     # Calculate coordinates for two points along the perpendicular line
    #     # Calculate the change in x and y based on the fixed horizontal distance
    #     delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
    #     delta_y = delta_x * slope_PAB
    #     # Calculate coordinates for two points along the perpendicular line
    #     left = Point(obs.p(t).x - delta_x, obs.p(t).y - delta_y)
    #     right = Point(obs.p(t).x + delta_x, obs.p(t).y + delta_y)
    #     # Cone
    #     cone_origin = Point(self.p(t).x, self.p(t).y)               
    #     cone = Polygon([cone_origin, left, right])
        
    #     P = Point(cone_origin.x + self.mydv(t).x, cone_origin.y + self.mydv(t).y)
    #     collision = P.within(cone) and self.p(t).distance(obs.p(t)) < SAFE_DIST
    #     if collision:
    #         time_collision_measure = self.p(t).distance(obs.p(t)) / math.sqrt(Vrel.x**2 + Vrel.y**2)
    #         steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
    #         risk = risk + 1/time_collision_measure + steering_effort_measure
                    
    #     return math.exp(risk)
    def risk(self, t, obs):
        """
        Risk

        Args:
            robot (RobotState): robot state
            people (TrackedPersons): tracked people
        """
        
        # risk = self.v[t]/20
        risk = np.random.normal(0, 0.02)
        collision = False
        
        # Calculate relative velocity vector
        Vrel = Point(obs.dv(t).x - self.dv(t).x, obs.dv(t).y - self.dv(t).y)
            
        # Compute the slope of the line AB and line PAB ⊥ AB
        slope_AB = (obs.p(t).y - self.p(t).y) / (obs.p(t).x - self.p(t).x)  # Rise over run
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
        collision = P.within(cone) and self.p(t).distance(obs.p(t)) < SAFE_DIST
        if collision:
            time_collision_measure = self.p(t).distance(obs.p(t)) / math.sqrt(Vrel.x**2 + Vrel.y**2)
            steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
            risk = risk + self.v[t] + 1/time_collision_measure + steering_effort_measure
                    
        return math.exp(risk)
       

if __name__ == '__main__': 
    DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/data'
    PP_DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
    CSV_NAME = ["data_20240131_234259", "data_20240131_234529"]
    for CSV in CSV_NAME:
        INPUT_CSV = DATA_DIR + '/' + CSV + '.csv'
        OUTPUT_CSV = PP_DATA_DIR + '/' + CSV + '.csv'
        
        OBS_SIZE = 2.0
        SAFE_DIST = 5.0

        # Read the CSV into a pandas DataFrame
        data = pd.read_csv(INPUT_CSV)
        noise_sz = data["r_x"].size
        R = Agent("R", data["r_x"] + np.random.normal(0,.05,noise_sz), data["r_y"] + np.random.normal(0,.05,noise_sz), data["r_{\theta}"], data["r_v"], data["r_{\omega}"], data["time"])
        H = Agent("H", data["h_x"] + np.random.normal(0,.05,noise_sz), data["h_y"] + np.random.normal(0,.05, noise_sz), data["h_{\theta}"], data["h_v"], data["h_{\omega}"], data["time"])
        # R = Agent("R", data["r_x"] + np.random.normal(0,.05,noise_sz), data["r_y"] + np.random.normal(0,.05,noise_sz), data["r_{\theta}"], data["r_v"] + np.random.normal(0,.05,noise_sz), data["r_{\omega}"], data["time"])
        # H = Agent("H", data["h_x"] + np.random.normal(0,.05,noise_sz), data["h_y"] + np.random.normal(0,.05, noise_sz), data["h_{\theta}"], data["h_v"]+ np.random.normal(0,.028,noise_sz), data["h_{\omega}"], data["time"])
        RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), data["time"])
        HG = Agent("HG", data["h_{gx}"], data["h_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), data["time"])
                
        df = pd.DataFrame(columns=["h_myv", "h_v", r"h_{\theta}", r"h_{\theta_{g}}", "h_my{d_g}", "h_{d_g}", "h_{risk}", r"h_{\omega}", r"h_{d_{obs}}"])  
        
        I0 = 2
        for i in range(I0, len(data)):                
            df.loc[i] = {"h_myv" : H.myv(i-1),
                        "h_v" : H.v[i],
                        r"h_{\theta}" : H.theta[i], 
                        r"h_{\theta_{g}}" : H.heading(i, HG),
                        "h_my{d_g}": H.mydist(i, HG),
                        "h_{d_g}" : H.dist(i, HG), 
                        "h_{risk}" : H.risk(i-1, R), 
                        r"h_{\omega}" : H.omega[i],
                        r"h_{d_{obs}}" : H.dist(i, R),
                        }

        # Save the processed data to another CSV file
        df = df[I0:]
        # df["h_myv"].plot()
        # df["h_v"].plot()
        # df["h_my{d_g}"].plot()
        # df["h_{d_g}"].plot()
        
        # plt.legend(["$myh_v$", "$h_v$", "$h_my{d_g}$", "$h_{d_g}$"])
        # plt.show()
        df.to_csv(OUTPUT_CSV, index=False)