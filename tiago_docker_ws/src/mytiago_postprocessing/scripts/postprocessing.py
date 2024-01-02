#!/usr/bin/env python

import os

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from move_base_msgs.msg import MoveBaseActionGoal
import message_filters
import rospy
import pandas as pd
from pedsim_msgs.msg import TrackedPersons
import tf


FILENAME = str(rospy.get_param("/mytiago_postprocessing/bagname"))
DATAPATH = str(rospy.get_param("/mytiago_postprocessing/datapath"))
NODE_NAME = 'mytiago_postprocessing'
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
        
        self.g_x = -5
        self.g_y = 5
        
        self.df = pd.DataFrame(columns=['g_x', 'g_y', 'r_x', 'r_y', 'r_theta', 'r_v', 'h_x', 'h_y', 'h_theta', 'h_v'])                           
        
        # Goal subscriber
        self.sub_goal = rospy.Subscriber('/move_base/goal', MoveBaseActionGoal, self.cb_goal)
        
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
                                                                queue_size = 100, slop = 1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.cb_handle_data)
        

    def cb_goal(self, goal: MoveBaseActionGoal):
        self.g_x = goal.goal.target_pose.pose.position.x
        self.g_y = goal.goal.target_pose.pose.position.y
                
                
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
        r_v = odom.twist.twist.linear.x
        # base_ang_vel = odom.twist.twist.angular.z
        
        
        p = people.tracks[0]            
        h_x = p.pose.pose.position.x
        h_y = p.pose.pose.position.y
        h_theta = p.pose.pose.orientation.z
        h_v = p.twist.twist.linear.x
                  
        # appending new data row in Dataframe
        self.df.loc[len(self.df)] = {'g_x': self.g_x, 'g_y': self.g_y,
                                     'r_x': r_x, 'r_y': r_y, 'r_theta': r_theta, 'r_v': r_v,
                                     'h_x': h_x, 'h_y':h_y, 'h_theta': h_theta, 'h_v': h_v,
                                    }                    

if __name__ == '__main__':
    os.makedirs(DATAPATH, exist_ok=True)
        
    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    rospy.spin()
            
    data_handler.df.to_csv(DATAPATH + "/" + FILENAME + "_raw.csv")