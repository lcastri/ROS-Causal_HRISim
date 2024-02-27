#!/usr/bin/env python

import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
import message_filters
import rospy
import pandas as pd
from pedsim_msgs.msg import TrackedPersons
from visualization_msgs.msg import Marker
import tf
import numpy as np


FILENAME = str(rospy.get_param("/hrisim_postprocessing/bagname"))
DATAPATH = str(rospy.get_param("/hrisim_postprocessing/datapath"))
NODE_NAME = 'hrisim_postprocessing'
NODE_RATE = 10 #Hz


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
            

class DataHandler():
    """
    Class handling data
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        self.std_dev = 0.05
        self.r_gx = -5
        self.r_gy = 5
        self.h_gx = 5
        self.h_gy = 5
        
        self.df = pd.DataFrame(columns=['time', 'r_gx', 'r_gy', 'h_gx', 'h_gy', 'r_x', 'r_y', 'r_theta', 'r_v', 'h_x', 'h_y', 'h_theta', 'h_v'])                           
        
        # Robot Goal subscriber
        self.sub_robot_goal = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_robot_goal)
        
        # Goal subscriber
        self.sub_human_goal = rospy.Subscriber('/goal_marker', Marker, self.cb_human_goal)
        
        # Odometry subscriber
        self.sub_odom = message_filters.Subscriber("/mobile_base_controller/odom", Odometry)
        
        # Person subscriber
        self.sub_people = message_filters.Subscriber('/ped/control/teleop_persons', TrackedPersons)
                
        # Robot pose subscriber
        self.sub_robot_pose = message_filters.Subscriber('/robot_pose', PoseWithCovarianceStamped)
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([self.sub_odom, 
                                                                self.sub_people, 
                                                                self.sub_robot_pose], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.cb_handle_data)
        

    def cb_robot_goal(self, goal: MoveBaseActionGoal):
        self.r_gx = goal.goal.target_pose.pose.position.x
        self.r_gy = goal.goal.target_pose.pose.position.y
        
        
    def cb_human_goal(self, goal: Marker):
        self.h_gx = goal.pose.position.x
        self.h_gy = goal.pose.position.y
                
                
    def cb_handle_data(self, odom: Odometry, 
                             people: TrackedPersons, 
                             robot_pose: PoseWithCovarianceStamped):
        """
        Synchronized callback

        Args:
            head_state (JointTrajectoryControllerState): robot head state
            torso_state (JointTrajectoryControllerState): robot torso state
            robot_odom (Odometry): robot odometry
            robot_pose (PoseWithCovarianceStamped): robot pose
            people (People): tracked people
        """

        # Robot 2D pose (x, y, theta)
        r_x, r_y, r_theta = get_2DPose(robot_pose)
        
        # Base linear & angular velocity
        r_v = abs(odom.twist.twist.linear.x)
        # base_ang_vel = odom.twist.twist.angular.z
        
        
        p = people.tracks[0]            
        h_x = p.pose.pose.position.x
        h_y = p.pose.pose.position.y
        h_theta = p.pose.pose.orientation.z
        h_v = abs(p.twist.twist.linear.x)
                  
        # appending new data row in Dataframe
        self.df.loc[len(self.df)] = {'time': odom.header.stamp.to_sec(),
                                     'r_gx': self.r_gx, 'r_gy': self.r_gy,
                                     'h_gx': self.h_gx, 'h_gy': self.h_gy,
                                     'r_x': r_x + np.random.normal(0, self.std_dev), 'r_y': r_y + np.random.normal(0, self.std_dev), 
                                     'r_theta': r_theta + np.random.normal(0, self.std_dev), 'r_v': r_v + np.random.normal(0, self.std_dev),
                                     'h_x': h_x + np.random.normal(0, self.std_dev), 'h_y':h_y + np.random.normal(0, self.std_dev),
                                     'h_theta': h_theta + np.random.normal(0, self.std_dev), 'h_v': h_v + np.random.normal(0, self.std_dev),
                                    }

if __name__ == '__main__':
    os.makedirs(DATAPATH, exist_ok=True)
        
    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    rospy.spin()
    
    data_handler.df = data_handler.df.assign(h_gx=data_handler.h_gx)
    data_handler.df = data_handler.df.assign(h_gy=data_handler.h_gy)
            
    data_handler.df.to_csv(DATAPATH + "/" + FILENAME + "_raw.csv")