from matplotlib import pyplot as plt
import numpy as np
import pandas as pd
from shapely.geometry import *
import math


class Agent():
    def __init__(self, name, x, y, theta, v, omega) -> None:
        self.name = name
        self.x = x
        self.y = y
        self.theta = theta
        self.v = v
        self.omega = omega
        
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
    
    def myomega(self, t, dt): #FIXME: remove me
        return (self.theta[t] - self.theta[t-1])/dt
    
    def myv(self, t, dt): #FIXME: remove me
        return math.sqrt((self.x[t] - self.x[t-1])**2 + (self.y[t] - self.y[t-1])**2)/dt
    
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

    def risk(self, t, obs):
        """
        risk of collision

        Args:
            t (int): time index
            obs (Agent): obstacle

        Returns:
            float: risk
        """
        
        risk = 0
        # risk = self.v[t]
        
        # Calculate relative velocity vector
        Vrel = Point(obs.dv(t).x - self.dv(t).x, obs.dv(t).y - self.dv(t).y)
               
        # Cone origin translated by Vb
        cone_origin = Point(self.p(t).x + obs.dv(t).x, self.p(t).y + obs.dv(t).y)
                            
        # Straight line from A to B = r_{a_b}
        AB = LineString([self.p(t), obs.p(t)])
            
        # PAB ⊥ AB passing through B
        left = AB.parallel_offset(5, 'left')
        right = AB.parallel_offset(5, 'right')

        c = left.boundary.geoms[1]
        d = right.boundary.geoms[0]
        PAB = LineString([c, d])
                    
        # Straight line ⊥ to r_{a_b} and passing through b
        B_encumbrance = obs.p(t).buffer(1.5)
        if PAB.intersects(B_encumbrance): 
            inter = PAB.intersection(B_encumbrance).xy
            inter_l = Point(inter[0][0] + obs.dv(t).x, inter[1][0] + obs.dv(t).y)
            inter_r = Point(inter[0][1] + obs.dv(t).x, inter[1][1] + obs.dv(t).y)
                    
            # Cone
            cone = Polygon([cone_origin, inter_l, inter_r])
            P = Point(cone_origin.x + Vrel.x, cone_origin.y + Vrel.y)
            if P.within(cone):
                time_collision_measure = self.dist(t, obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
                steering_effort_measure = min(P.distance(LineString([cone_origin, inter_l])), P.distance(LineString([cone_origin, inter_r])))           
                risk = risk + 1/time_collision_measure + steering_effort_measure
                    
        # return math.exp(risk)
        return risk

       

if __name__ == '__main__': 
       
    DATA_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/data'
    POSTPROCESS_DIR = '~/git/TIAGo-docker/utilities_ws/src/causal_discovery_offline/ppdata'
    csv_name = 'raw20240125_154606'
    # csv_name = 'raw20240125_155709'
    INPUT_CSV = DATA_DIR + '/' + csv_name + '.csv'
    OUTPUT_CSV = POSTPROCESS_DIR + '/' + csv_name + '.csv'

    STD = [0.01, 0.05, 0.09, 0.02, 
           0.01, 0.04, 0.02, 0.045]
    
    # Read the CSV into a pandas DataFrame
    data = pd.read_csv(INPUT_CSV)
    
    nsize = data["r_x"].size
    R = Agent("R", data["r_x"] + np.random.normal(0, STD[0], nsize),
                   data["r_y"] + np.random.normal(0, STD[0], nsize), 
                   data["r_{\theta}"] + np.random.normal(0, STD[1], nsize), 
                   data["r_v"] + np.random.normal(0, STD[0], nsize), 
                   data["r_{\omega}"] + np.random.normal(0, STD[1], nsize)) 
    H = Agent("H", data["h_x"] + np.random.normal(0, STD[4], nsize), 
                   data["h_y"] + np.random.normal(0, STD[4], nsize),
                   data["h_{\theta}"] + np.random.normal(0, STD[5], nsize),
                   data["h_v"] + np.random.normal(0, STD[4], nsize), 
                   data["h_{\omega}"] + np.random.normal(0, STD[5], nsize))
    RG = Agent("RG", data["r_{gx}"], data["r_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
    HG = Agent("HG", data["h_{gx}"], data["h_{gy}"], np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]), np.zeros_like(data["r_{gy}"]))
            
    df = pd.DataFrame(columns=["r_v", "r_{d_g}", "r_{risk}", r"r_{d_{obs}}", r"r_{\theta}", r"r_{\theta_{gr}}", r"r_{\theta_{g}}", r"r_{\omega}", 
                               "h_v", "h_{d_g}", "h_{risk}", r"h_{d_{obs}}", r"h_{\theta}", r"h_{\theta_{gr}}", r"h_{\theta_{g}}", r"h_{\omega}"])
            
            
    for i in range(1, len(data)):
        dt = data['time'].loc[i] - data['time'].loc[i-1]
        df.loc[i] = {"r_v" : R.v[i],
                     "r_{d_g}" : R.dist(i, RG),
                     "r_{risk}" : R.risk(i-1, H),
                     r"r_{d_{obs}}" : R.dist(i, H),
                     r"r_{\theta}" : R.theta[i],
                     r"r_{\theta_{gr}}" : R.heading(i, RG), 
                     r"r_{\theta_{g}}" : R.bearing(i, RG), 
                     r"r_{\omega}" : R.omega[i],
                    #  "h_v" : H.v[i],
                     "h_v" : H.myv(i, dt),
                     "h_{d_g}" : H.dist(i, HG), 
                     "h_{risk}" : H.risk(i-1, R), 
                     r"h_{d_{obs}}" : H.dist(i, R),
                     r"h_{\theta}" : H.theta[i], 
                     r"h_{\theta_{gr}}" : H.heading(i, HG), 
                     r"h_{\theta_{g}}" : H.bearing(i, HG), 
                     r"h_{\omega}" : H.omega[i],
                     }

    # Save the processed data to another CSV file
    df = df[1:-1]
    df[r"h_v"].plot()
    df[r"h_{d_g}"].plot()
    df[r"h_{risk}"].plot()
    df[r"h_{\theta}"].plot()
    df[r"h_{\theta_{g}}"].plot()
    # df[r"h_{d_{obs}}"].plot()
    plt.legend([r"$h_{\theta_{gr}}$", r"$h_{\omega}$", r"$h_{risk}$", "$h_v$", "$h_{d_g}$"])

    plt.show()
    df.to_csv(OUTPUT_CSV, index=False)