#!/usr/bin/env python

from datetime import datetime
import numpy as np
import rospy
import pandas as pd
import os
import message_filters
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from pedsim_msgs.msg import TrackedPersons
from mytiago_human.msg import Risk, HumanState
from mytiago_robot.msg import RobotState


NODE_NAME = "mytiago_data_collector"
NODE_RATE = 10 # [Hz]
TS_LENGTH = float(rospy.get_param("/mytiago_data_collector/ts_length", default = 150)) # [s]
DATA_DIR = str(rospy.get_param("/mytiago_data_collector/data_dir", default = '/root/shared/')) + 'data_pool'
DT = float(rospy.get_param("/mytiago_data_collector/dt", default = 0.1))
SUBSAMPLING = bool(rospy.get_param("/mytiago_data_collector/subsampling", default = False))
ADDNOISE = bool(rospy.get_param("/mytiago_data_collector/addnoise", default = False))
STD = [float(x) for x in rospy.get_param('/mytiago_data_collector/std').split(',')]
# STD = float(rospy.get_param("/mytiago_data_collector/std", default = 0.05))
ID_FORMAT = str(rospy.get_param("/mytiago_data_collector/id_format", default = '%Y%m%d_%H%M%S'))
CSV_PREFIX = str(rospy.get_param("/mytiago_data_collector/cas_prefix", default = 'data_'))


class DataCollector():

    def __init__(self) -> None:
        """
        DataCollector constructor
        """
        self.df = None
        self.raw = None
        self.time_init = None
        self.rg = (None, None)
        self.hg = (None, None)
        
        # Robot Goal subscriber
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        
        # Goal subscriber
        rospy.Subscriber('/goal_marker', Marker, self.cb_human_goal)
                 
        # Robot  subscriber
        sub_robot = message_filters.Subscriber("/hri/robot_state", RobotState)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/hri/human_state', HumanState)
                        
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.cb_handle_data)
                    

    def cb_robot_goal(self, goal: MoveBaseActionGoal):
        """
        Robot goal callback

        Args:
            goal (MoveBaseActionGoal): robot goal
        """
        self.rg = (goal.goal.target_pose.pose.position.x, goal.goal.target_pose.pose.position.y)
        
        
    def cb_human_goal(self, goal: Marker):
        """
        Human goal callback

        Args:
            goal (Marker): human goal
        """
        self.hg = (goal.pose.position.x, goal.pose.position.y)
                
                
    def cb_handle_data(self, robot: RobotState, human: HumanState):
        """
        Synchronized callback

        Args:
            robot (RobotState): robot state
            human (HumanState): tracked person
        """
        time_now = robot.header.stamp.to_sec()
        if self.time_init is None or (time_now - self.time_init >= TS_LENGTH):
            
            # Save currect dataframe
            if self.df is not None:
                # this is to make sure that the dataframe contains data with time in ascending order.
                # with ApproximateTimeSynchronizer might not be always true
                self.df.sort_values(by = 'time', ascending = True, inplace = True, ignore_index = True)
                self.raw.sort_values(by = 'time', ascending = True, inplace = True, ignore_index = True)
                
                # subsample your dataset
                if SUBSAMPLING: self.df = self.subsampling(self.df, DT)
                if SUBSAMPLING: self.raw = self.subsampling(self.raw, DT)
                timestamp_str = datetime.now().strftime(ID_FORMAT)

                rospy.logwarn("CSV file saved: " + CSV_PREFIX + timestamp_str + '.csv')
                self.df.to_csv(DATA_DIR + '/' + CSV_PREFIX + timestamp_str + '.csv', index=False)
                self.raw.to_csv(DATA_DIR + '/raw' + timestamp_str + '.csv', index=False)

            
            # Init dataframe
            columns = ['time', 'r_{gx}', 'r_{gy}', 'r_x', 'r_y', 'r_{\theta}', 'r_v', 'r_{\omega}', 'h_{gx}', 'h_{gy}', 'h_x', 'h_y', 'h_{\theta}', 'h_v', 'h_{\omega}', 'h_{risk}', 'h_{d_g}', 'h_{\theta_g}']
            self.df = pd.DataFrame(columns=columns)
            self.raw = pd.DataFrame(columns=columns)
            self.time_init = time_now

        # Robot 2D pose (x, y, theta) and velocity
        r_x = robot.pose2D.x
        r_y = robot.pose2D.y
        r_theta = robot.pose2D.theta
        r_v = robot.twist.linear.x
        r_w = robot.twist.angular.z
        
        # Human 2D pose (x, y, theta) and velocity            
        h_x = human.pose2D.x
        h_y = human.pose2D.y
        h_theta = human.pose2D.theta
        h_v = human.twist.linear.x
        h_w = human.twist.angular.z
        h_risk = human.risk.risk.data
        h_dg = human.dg.data
        h_thetag = human.thetag.data
                  
        # appending new data row in Dataframe
        nrx = np.random.normal(0, STD[0]) if ADDNOISE else 0
        nry = np.random.normal(0, STD[0]) if ADDNOISE else 0
        nrtheta = np.random.normal(0, STD[1]) if ADDNOISE else 0
        nrv = np.random.normal(0, STD[2]) if ADDNOISE else 0
        nrw = np.random.normal(0, STD[3]) if ADDNOISE else 0
        nhx = np.random.normal(0, STD[4]) if ADDNOISE else 0
        nhy = np.random.normal(0, STD[4]) if ADDNOISE else 0
        nhtheta = np.random.normal(0, STD[5]) if ADDNOISE else 0
        nhv = np.random.normal(0, STD[6]) if ADDNOISE else 0
        nhw = np.random.normal(0, STD[7]) if ADDNOISE else 0
        
        if self.rg == (None, None): self.rg = (r_x, r_y)
        if self.hg == (None, None): self.hg = (h_x, h_y)
        self.df.loc[len(self.df)] = {'time': robot.header.stamp.to_sec(),
                                     'r_{gx}': self.rg[0], 'r_{gy}': self.rg[1],
                                     'r_x': r_x + nrx, 'r_y': r_y + nry, 
                                     'r_{\theta}': r_theta + nrtheta, 'r_v': r_v + nrv, 'r_{\omega}': r_w + nrw,
                                     'h_{gx}': self.hg[0], 'h_{gy}': self.hg[1],
                                     'h_x': h_x + nhx, 'h_y':h_y + nhy,
                                     'h_{\theta}': h_theta + nhtheta, 'h_v': h_v + nhv, 'h_{\omega}': h_w + nhw, 
                                     'h_{risk}': h_risk, 'h_{d_g}': h_dg, 'h_{\theta_g}': h_thetag
                                    }
        self.raw.loc[len(self.raw)] = {'time': robot.header.stamp.to_sec(),
                                     'r_{gx}': self.rg[0], 'r_{gy}': self.rg[1],
                                     'r_x': r_x, 'r_y': r_y, 
                                     'r_{\theta}': r_theta, 'r_v': r_v, 'r_{\omega}': r_w,
                                     'h_{gx}': self.hg[0], 'h_{gy}': self.hg[1],
                                     'h_x': h_x, 'h_y': h_y,
                                     'h_{\theta}': h_theta, 'h_v': h_v, 'h_{\omega}': h_w, 
                                     'h_{risk}': h_risk, 'h_{d_g}': h_dg, 'h_{\theta_g}': h_thetag
                                    }
        
        
    def subsampling(self, df: pd.DataFrame, dt, tol = 0.01):
        """
        subsampling the dataframe taking a sample each dt secs
        
        Args:
            dt (float): subsampling step
            tol (float): tolerance
            
        Returns:
            pd.DataFrame: subsampled dataframe
        """
        sd = pd.DataFrame(columns=df.columns)
        sd.loc[0] = df.loc[0]
        init_t = df.time.values[0]
        for i in range(1, len(df)):
            if df.time.values[i] - init_t >= dt - tol:
                sd.loc[i] = df.loc[i]
                init_t = df.time.values[i]
        return sd.reset_index(drop=True)
        

if __name__ == '__main__':

    # Create data pool directory
    os.makedirs(DATA_DIR, exist_ok=True)
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    dc = DataCollector()

    while not rospy.is_shutdown():
        rate.sleep()