#!/usr/bin/env python

import math
import os
import sys
import json
import pandas as pd
from shapely.geometry import Polygon, Point
import rospy
from geometry_msgs.msg import PoseWithCovariance
from people_msgs.msg import People
from tf.transformations import quaternion_from_euler


NODE_NAME = 'extract_goal'
NODE_RATE = 10 #Hz
MAP_BOUNDARIES = [(5.45, -4.66), (0.75, -0.56), (0.47, -0.73), (-0.73, 0.28), 
                  (0.01, 1.05), (-0.37, 1.37), (0.69, 2.62), (1.29, 2.14),
                  (2.01, 2.82), (2.95, 1.82), (3.52, 1.4), (4.16, 1.17), (7.86, -1.85)]
MAP = Polygon(MAP_BOUNDARIES)       

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
        self.old_goal = None
        with open(PEOPLE_ID) as json_file:
            peopleID = json.load(json_file)
            self.A_ID = peopleID[AGENT]
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
        for p in people.people:
            if int(p.name) in self.A_ID:
                
                pose = self.extract_pose(p)
                
                # check if traj point is contained in the map 
                # if not MAP.contains(Point(pose.pose.position.x, pose.pose.position.y)):
                #     continue
                 
                if int(p.name) != self.selHID:
                    self.old_pos = None
                    self.prev_time = None
                self.selHID = int(p.name)
                
                if self.old_pos is None or self.prev_time is None:
                    self.prev_time = people.header.stamp
                    self.old_pos = self.extract_pose(p)
                    return
                
                goal = None
                for gid, g in GOALS.items():
                    if Point(pose.pose.position.x, pose.pose.position.y).distance(Point(g["x"], g["y"])) <= g["radius"]:
                        goal = Point(g["x"], g["y"])
                        rospy.logwarn("GOAL: " + gid)
                if goal == self.old_goal: goal = None
                self.raw.loc[len(self.raw)] = {'time': people.header.stamp.to_sec(), 
                                               'h_{gx}': goal.x if goal is not None else None, 
                                               'h_{gy}': goal.y if goal is not None else None,}
                if goal is not None: self.old_goal = goal
                return
                  

if __name__ == '__main__': 
    
    AGENT = sys.argv[1]
    PEOPLE_ID = sys.argv[2]
    with open(sys.argv[3]) as json_file:
        GOALS = json.load(json_file)
    
    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    def cleanup():
        original_path = "~/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data"
        csv_path = os.path.expanduser(original_path)
        data_handler.raw.bfill(inplace=True)
        data_handler.raw.to_csv(csv_path + '/' + AGENT + "_goal.csv", sep=',', index=False)

    rospy.on_shutdown(cleanup)

    while not rospy.is_shutdown():
        rate.sleep()

    cleanup()