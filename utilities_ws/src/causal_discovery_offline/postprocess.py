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
       

if __name__ == '__main__': 
       
    DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/data'
    POSTPROCESS_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
    CSV_NAME = 'raw20240128_171705'
    INPUT_CSV = DATA_DIR + '/' + CSV_NAME + '.csv'
    OUTPUT_CSV = POSTPROCESS_DIR + '/' + CSV_NAME + '.csv'

    RSTD = [0.05, 0.0, 0.0]
    HSTD = [0.015, 0.01, 0.0]

    
    # Read the CSV into a pandas DataFrame
    data = pd.read_csv(INPUT_CSV)
    
    nsize = data["r_x"].size
    
    R = Agent("R", data["r_x"] + np.random.normal(0, RSTD[0], nsize),
                   data["r_y"] + np.random.normal(0, RSTD[0], nsize), 
                   data["r_{\theta}"] + np.random.normal(0, RSTD[1], nsize), 
                   abs(data["r_v"]) + np.random.normal(0, RSTD[0], nsize), 
                   data["r_{\omega}"] + np.random.normal(0, RSTD[1], nsize)) 
    
    H = Agent("H", data["h_x"] + np.random.normal(0, HSTD[0], nsize), 
                   data["h_y"] + np.random.normal(0, HSTD[0], nsize),
                   data["h_{\theta}"] + np.random.normal(0, HSTD[1], nsize),
                   abs(data["h_v"]) + np.random.normal(0, HSTD[0], nsize), 
                   data["h_{\omega}"] + np.random.normal(0, HSTD[1], nsize),
                   data["h_{risk}"] + np.random.normal(0, HSTD[0], nsize))
    RG = Agent("RG", data["r_{gx}"], data["r_{gy}"])
    HG = Agent("HG", data["h_{gx}"], data["h_{gy}"])
            
    df = pd.DataFrame(columns=["h_v", "h_{d_g}", "h_{risk}"])
            
            
    for i in range(1, len(data)):
        df.loc[i] = {"h_v" : H.v[i],
                     "h_{d_g}" : H.dist(i, RG), 
                     "h_{risk}" : H.risk[i], 
                     }

    # Save the processed data to another CSV file
    df = df[1:-1]
    df[r"h_v"].plot()
    df[r"h_{d_g}"].plot()
    df[r"h_{risk}"].plot()
    plt.legend(["$h_v$", "$h_{d_g}$", r"$h_{risk}$"])

    plt.show()
    df.to_csv(OUTPUT_CSV, index=False)