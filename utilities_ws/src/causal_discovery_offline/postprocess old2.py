from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from shapely.geometry import *
import math


class Agent():
    def __init__(self, name, x, y, theta = None, v = None, omega = None, risk = None, dg = None, thetag = None) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.theta = np.zeros_like(x) if theta is None else theta
        self.v = np.zeros_like(x) if v is None else v
        self.omega = np.zeros_like(x) if omega is None else omega
        self.risk = np.zeros_like(x) if risk is None else risk
        self.dg = np.zeros_like(x) if dg is None else dg
        self.thetag = np.zeros_like(x) if thetag is None else thetag
        
    def p(self, t):
        """
        Position

        Args:
            t (int): time index

        Returns:
            Point: position
        """
        return Point(self.x[t], self.y[t])
    
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
    
    def bearing(self, t, obs):
        """
        Bearing angle

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: bearing angle
        """
        return math.atan2(obs.p(t).y - self.p(t).y, obs.p(t).x - self.p(t).x)
       
    def heading(self, t, obs):
        """
        heading angle

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: bearing angle
        """
        current_direction = np.array([np.cos(self.theta[t]), np.sin(self.theta[t])])
        current_direction /= np.sqrt(np.sum(current_direction**2))  # normalize the vector

        target_direction = np.array([obs.p(t).x - self.p(t).x, obs.p(t).y - self.p(t).y])
        target_direction /= np.sqrt(np.sum(target_direction**2))  # normalize the vector

        delta = np.arctan2(np.linalg.norm(np.cross(target_direction, current_direction)),
                           np.dot(target_direction, current_direction))
        return delta
       

if __name__ == '__main__': 
       
    DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/data'
    POSTPROCESS_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
    CSV_NAME = 'raw20240128_164623'
    INPUT_CSV = DATA_DIR + '/' + CSV_NAME + '.csv'
    OUTPUT_CSV = POSTPROCESS_DIR + '/' + CSV_NAME + '.csv'

    STD = [0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0]
    # STD = [0.01, 0.05, 0.09, 0.02, 
    #        0.01, 0.04, 0.02, 0.045]
    
    # Read the CSV into a pandas DataFrame
    data = pd.read_csv(INPUT_CSV)
    
    nsize = data["r_x"].size
    
    R = Agent("R", data["r_x"] + np.random.normal(0, STD[0], nsize),
                   data["r_y"] + np.random.normal(0, STD[0], nsize), 
                   data["r_{\theta}"] + np.random.normal(0, STD[1], nsize), 
                   abs(data["r_v"]) + np.random.normal(0, STD[0], nsize), 
                   data["r_{\omega}"] + np.random.normal(0, STD[1], nsize)) 
    
    H = Agent("H", data["h_x"] + np.random.normal(0, STD[4], nsize), 
                   data["h_y"] + np.random.normal(0, STD[4], nsize),
                   data["h_{\theta}"] + np.random.normal(0, STD[5], nsize),
                   abs(data["h_v"]) + np.random.normal(0, STD[4], nsize), 
                   data["h_{\omega}"] + np.random.normal(0, STD[5], nsize),
                   data["h_{risk}"] + np.random.normal(0, STD[4], nsize),
                   data["h_{d_g}"] + np.random.normal(0, STD[4], nsize),
                   data["h_{\theta_g}"] + np.random.normal(0, STD[5], nsize))
    RG = Agent("RG", data["r_{gx}"], data["r_{gy}"])
    HG = Agent("HG", data["h_{gx}"], data["h_{gy}"])
            
    df = pd.DataFrame(columns=["r_v", "r_{d_g}", "r_{risk}", r"r_{d_{obs}}", r"r_{\theta}", r"r_{\theta_{gr}}", r"r_{\theta_{g}}", r"r_{\omega}", 
                               "h_v", "h_{d_g}", "h_{risk}", r"h_{d_{obs}}", r"h_{\theta}", r"h_{\theta_{gr}}", r"h_{\theta_{g}}", r"h_{\omega}"])
            
            
    for i in range(1, len(data)):
        dt = data['time'].loc[i] - data['time'].loc[i-1]
        df.loc[i] = {"r_v" : R.v[i],
                     "r_{d_g}" : R.dist(i, RG),
                     "r_{risk}" : R.risk[i],
                     r"r_{d_{obs}}" : R.dist(i, H),
                     r"r_{\theta}" : R.theta[i],
                     r"r_{\theta_{gr}}" : R.heading(i, RG), 
                     r"r_{\theta_{g}}" : R.bearing(i, RG), 
                     r"r_{\omega}" : R.omega[i],
                     "h_v" : H.v[i],
                     "h_{d_g}" : H.dg[i], 
                     "h_{risk}" : H.risk[i], 
                     r"h_{d_{obs}}" : H.dist(i, R),
                     r"h_{\theta}" : H.theta[i], 
                     r"h_{\theta_{gr}}" : H.thetag[i], 
                     r"h_{\theta_{g}}" : H.bearing(i, HG), 
                     r"h_{\omega}" : H.omega[i],
                     }

    # Save the processed data to another CSV file
    df = df[1:-1]
    df[r"h_v"].plot()
    df[r"h_{d_g}"].plot()
    df[r"h_{risk}"].plot()
    df[r"h_{\omega}"].plot()
    df[r"h_{\theta_{gr}}"].plot()
    plt.legend(["$h_v$", "$h_{d_g}$", r"$h_{risk}$", r"$h_{\omega}$", r"$h_{\theta_{gr}}$"])

    plt.show()
    df.to_csv(OUTPUT_CSV, index=False)