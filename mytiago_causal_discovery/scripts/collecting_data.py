#!/usr/bin/env python


from datetime import datetime
import numpy as np
import rospy
import pandas as pd
import os
import message_filters
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped
import tf
from pedsim_msgs.msg import TrackedPersons


NODE_NAME = "mytiago_data_collector"
NODE_RATE = 10 # [Hz]
TS_LENGTH = float(rospy.get_param("/mytiago_data_collector/ts_length", default = 150)) # [s]
DATA_DIR = str(rospy.get_param("/mytiago_data_collector/data_dir", default = '/root/shared/')) + 'data_pool'
DT = float(rospy.get_param("/mytiago_data_collector/dt", default = 0.1))
SUBSAMPLING = bool(rospy.get_param("/mytiago_data_collector/subsampling", default = False))
ADDNOISE = bool(rospy.get_param("/mytiago_data_collector/addnoise", default = False))
STD = bool(rospy.get_param("/mytiago_data_collector/std", default = 0.05))
ID_FORMAT = str(rospy.get_param("/mytiago_data_collector/id_format", default = '%Y%m%d_%H%M%S'))
CSV_PREFIX = str(rospy.get_param("/mytiago_data_collector/cas_prefix", default = 'data_'))


def get_2DPose(p: PoseWithCovarianceStamped):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (PoseWithCovarianceStamped): pose
    
    Returns:
        tuple: x, y, theta
    """
    x = p.pose.pose.position.x
    y = p.pose.pose.position.y
    
    q = (
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w
    )
    
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return x, y, yaw


class DataCollector():

    def __init__(self) -> None:
        """
        DataCollector constructor
        """
        self.df = None
        self.time_init = None
        self.rg = (None, None)
        self.hg = (None, None)
        
        # Robot Goal subscriber
        rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        
        # Goal subscriber
        rospy.Subscriber('/goal_marker', Marker, self.cb_human_goal)
        
        # Odometry subscriber
        sub_odom = message_filters.Subscriber("/mobile_base_controller/odom", Odometry)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/ped/control/teleop_persons', TrackedPersons)
                
        # Robot pose subscriber
        sub_robot_pose = message_filters.Subscriber('/robot_pose', PoseWithCovarianceStamped)
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_odom, 
                                                                sub_people, 
                                                                sub_robot_pose], 
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
                
                
    def cb_handle_data(self, odom: Odometry, 
                             people: TrackedPersons, 
                             robot_pose: PoseWithCovarianceStamped):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            people (TrackedPersons): tracked people
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        time_now = odom.header.stamp.to_sec()
        if self.time_init is None or (time_now - self.time_init >= TS_LENGTH):
            
            # Save currect dataframe
            if self.df is not None:
                # this is to make sure that the dataframe contains data with time in ascending order.
                # with ApproximateTimeSynchronizer might not be always true
                self.df.sort_values(by = 'time', ascending = True, inplace = True, ignore_index = True)
                
                # subsample your dataset
                if SUBSAMPLING: self.df = self.subsampling(DT)
                timestamp_str = datetime.now().strftime(ID_FORMAT)

                self.df.to_csv(DATA_DIR + '/' + CSV_PREFIX + timestamp_str + '.csv', index=False)
            
            # Init dataframe    
            self.df = pd.DataFrame(columns=['time', 'r_{gx}', 'r_{gy}', 'r_x', 'r_y', 'r_{\theta}', 'r_v', 'r_{\omega}', 'h_{gx}', 'h_{gy}', 'h_x', 'h_y', 'h_{\theta}', 'h_v', 'h_{\omega}'])
            self.time_init = time_now

        # Robot 2D pose (x, y, theta) and velocity
        r_x, r_y, r_theta = get_2DPose(robot_pose)
        r_v = abs(odom.twist.twist.linear.x)
        r_omega = abs(odom.twist.twist.angular.z)
        
        # Human 2D pose (x, y, theta) and velocity
        p = people.tracks[0]            
        h_x = p.pose.pose.position.x
        h_y = p.pose.pose.position.y
        h_theta = p.pose.pose.orientation.z
        h_v = abs(p.twist.twist.linear.x)
        h_omega = abs(p.twist.twist.angular.z)
                  
        # appending new data row in Dataframe
        noise = np.zeros(10)
        if ADDNOISE: noise = np.random.normal(0, STD, size = 10)
        if self.rg == (None, None): self.rg = (r_x, r_y)
        if self.hg == (None, None): self.hg = (h_x, h_y)
        self.df.loc[len(self.df)] = {'time': odom.header.stamp.to_sec(),
                                     'r_{gx}': self.rg[0], 'r_{gy}': self.rg[1],
                                     'r_x': r_x + noise[0], 'r_y': r_y + noise[1], 
                                     'r_{\theta}': r_theta + noise[2], 'r_v': r_v + noise[3], 'r_{\omega}': r_omega + noise[4],
                                     'h_{gx}': self.hg[0], 'h_{gy}': self.hg[1],
                                     'h_x': h_x + noise[5], 'h_y':h_y + noise[6],
                                     'h_{\theta}': h_theta + noise[7], 'h_v': h_v + noise[8], 'h_{\omega}': h_omega + noise[9],
                                    }
        
        
    def subsampling(self, dt, tol = 0.01):
        """
        subsampling the dataframe taking a sample each dt secs
        
        Args:
            dt (float): subsampling step
            tol (float): tolerance
            
        Returns:
            pd.DataFrame: subsampled dataframe
        """
        sd = pd.DataFrame(columns=self.df.columns)
        sd.loc[0] = self.df.loc[0]
        init_t = self.df.time.values[0]
        for i in range(1, len(self.df)):
            if self.df.time.values[i] - init_t >= dt - tol:
                sd.loc[i] = self.df.loc[i]
                init_t = self.df.time.values[i]
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