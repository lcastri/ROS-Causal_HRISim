#!/usr/bin/env python

import math
import os
import sys

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import rospy
import pandas as pd
from people_msgs.msg import People
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
import tf
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from shapely.geometry import Polygon, Point


NODE_NAME = 'extract_sel_human'
NODE_RATE = 10 #Hz
MAP_BOUNDARIES = [(5.04, -5.28), (-1.156, -0.189), (1.92, 3.03), (7.88, -1.76)]
MAP = Polygon(MAP_BOUNDARIES)       
GOAL_PARAM = "/hri/goal"
GOAL_LIST = [(3.5, -2.5), (-0.295, 0.386), (1.878, 2.371), (7.069, -1.907)]

class DataHandler():
    """
    Class handling data
    """
    
    def __init__(self):
        """
        Class constructor. Init publishers and subscribers
        """
        self.old_pos = None
        self.prev_time = None
        self.selHID = None
        
        columns = ['time', 'h_{gx}', 'h_{gy}']
        self.raw = pd.DataFrame(columns=columns)

        # Person subscriber
        self.sub_people = rospy.Subscriber('/people_tracker/people', People, self.cb_goal)
        
        
    def extract_pose(self, p):
        pose = PoseWithCovariance()
        pose.pose.position.x = p.position.x
        pose.pose.position.y = p.position.y
        pose.pose.position.z = p.position.z
        (x, y, z, w) = quaternion_from_euler(0, 0, math.atan2(p.velocity.y, p.velocity.x))
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        pose.covariance = [0.0] * 36
        return pose
                                                    
                
    def cb_goal(self, people: People):
        """
        People callback

        Args:
            people (People): tracked people
        """
        
        # SEL_H_ID = [103, 235, 267, 315] # run0.bag
        SEL_H_ID = [144, 159,165,177,189, 191, 203, 210, 
                    217, 223, 233, 247, 260, 268, 282, 
                    288, 291, 300,313, 345, 357, 367, 
                    374, 376, 397,401,413, 433, 436, 448, 
                    450, 454, 468, 473] # runLuca.bag
        for p in people.people:
            if int(p.name) in SEL_H_ID:
                
                pose = self.extract_pose(p)
                
                # check if traj point is contained in the map 
                if not MAP.contains(Point(pose.pose.position.x, pose.pose.position.y)):
                    continue
                 
                if int(p.name) != self.selHID:
                    self.old_pos = None
                    self.prev_time = None
                self.selHID = int(p.name)
                
                if self.old_pos is None or self.prev_time is None:
                    self.prev_time = people.header.stamp
                    self.old_pos = self.extract_pose(p)
                    return
                
                goal = None
                for g in GOAL_LIST:
                    if Point(pose.pose.position.x, pose.pose.position.y).distance(Point(g[0],g[1])) <= 1.25:
                        goal = Point(g[0], g[1])
                        rospy.logwarn("GOAL: " + str(goal))
                        
                self.raw.loc[len(self.raw)] = {'time': people.header.stamp.to_sec(), 
                                               'h_{gx}': goal.x if goal is not None else None, 
                                               'h_{gy}': goal.y if goal is not None else None,}
                return
                  

if __name__ == '__main__': 
    
    csv_name = sys.argv[1]
    
    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    def cleanup():
        original_path = "~/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data"
        csv_path = os.path.expanduser(original_path)
        data_handler.raw.bfill(inplace=True)
        data_handler.raw.to_csv(csv_path + '/' + csv_name + ".csv", sep=',', index=False)

    rospy.on_shutdown(cleanup)

    while not rospy.is_shutdown():
        rate.sleep()

    cleanup()