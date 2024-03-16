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
    def __init__(self, name, x, y, time, theta = None, v = None, omega = None):
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        self.time = time
        
        self.gr = False
        self.rotating = False
        
    def goal_reached(self, t, goal):
        if goal.p(t) != goal.p(t-1): 
            return 1
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
        # risk = self.v[t] / self.dist(t, obs)
        # risk = self.v[t]
        risk = self.v[t]/10 + np.random.normal(0, 0.02)
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
        collision = P.within(cone) and self.dist(t, obs) < SAFE_DIST
        if collision:
            time_collision_measure = self.dist(t, obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
            steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
            # risk = risk + 1/time_collision_measure + steering_effort_measure
            risk = risk + 1/time_collision_measure + steering_effort_measure
                    
        # return math.exp(risk)
        return math.exp(risk)
       

if __name__ == '__main__': 
    DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/Traj'
    PP_DATA_DIR = '~/git/ROS-Causal_HRISim/utilities_ws/src/causal_discovery_offline/Traj_pp'
    AGENT = ["A" + str(i) for i in range(1, 16)]
    # LENGTH = 1500
    dfs = list()
    for A in AGENT:
        INPUT_CSV = DATA_DIR + '/' + A + '_traj.csv'
        OUTPUT_CSV = PP_DATA_DIR + '/' + A + '.csv'
        
        OBS_SIZE = 2.0
        SAFE_DIST = 5
        SEL_ID = "1000_"

        # Read the CSV into a pandas DataFrame
        data = pd.read_csv(INPUT_CSV)
        # data.dropna(inplace=True)
        # data.reset_index(drop=True, inplace=True)
        # noise_sz = data["r_x"].size
        R = Agent("R", data["r_x"], data["r_y"], data["time"], data[r"r_{\theta}"], data["r_v"], data["r_{\omega}"])
        H = Agent("H", data["h_" + SEL_ID + "x"], data["h_" + SEL_ID + "y"], data["time"], data["h_" + SEL_ID + r"{\theta}"], data["h_" + SEL_ID + "v"], data["h_" + SEL_ID + "{\omega}"])
        RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], data["time"])
        HG = Agent("HG", data["h_" + SEL_ID + "{gx}"], data["h_" + SEL_ID + "{gy}"], data["time"])
                
        df = pd.DataFrame(columns=["v", "d_g", "r"])  
        # df = pd.DataFrame(columns=["g_r", "v", r"\theta_{g}", "d_g", "r", r"\omega", r"d_{obs}"])  
        
        I0 = 1
        for i in range(I0, len(data)-1):
            df.loc[i] = {
                        "v" : H.v[i],
                        "d_g" : H.dist(i, HG),
                        "r" : H.risk(i, R), 
                        }
            # df.loc[i] = {
            #             "g_r": H.goal_reached(i+1, HG),
            #             "v" : abs(H.v[i]),
            #             "d_g" : H.dist(i, HG),
            #             r"\theta_{g}" : H.heading(i, HG),
            #             "r" : H.risk(i-1, R), 
            #             r"\omega" : H.omega[i],
            #             r"d_{obs}" : H.dist(i, R),
            #             }

        # Save the processed data to another CSV file
        df['r'] = df['r'].shift(-1)
        df.loc[len(df)-1, 'r'] = 0
        
        df = df[I0:-1]
        # df = df[:LENGTH]
        df.to_csv(OUTPUT_CSV, index = False)
        dfs.append(df)
        
    fdf = pd.concat(dfs, ignore_index=True, sort=False)
    columns=["v", "d_g", "r"]
    fdf[columns].plot()
    legend = ['$' + k + '$' for k in columns]
    plt.legend(legend)
    plt.show()