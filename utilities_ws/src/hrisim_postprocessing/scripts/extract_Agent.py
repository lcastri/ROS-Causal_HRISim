#!/usr/bin/env python

import math
import os
import sys

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import rospy
import pandas as pd
from people_msgs.msg import People
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
import json
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from shapely.geometry import Polygon, Point


NODE_NAME = 'extract_agent'
NODE_RATE = 10 #Hz
MAP_BOUNDARIES = [(5.45, -4.66), (0.75, -0.56), (0.47, -0.73), (-0.73, 0.28), 
                  (0.01, 1.05), (-0.37, 1.37), (0.69, 2.62), (1.29, 2.14),
                  (2.01, 2.82), (2.95, 1.82), (3.52, 1.4), (4.16, 1.17), (7.86, -1.85)]
MAP = Polygon(MAP_BOUNDARIES)       
GOAL_PARAM = "/hri/goal"

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
        
        with open(PEOPLE_ID) as json_file:
            peopleID = json.load(json_file)
            self.A_ID = peopleID[AGENT]
        
        original_path = "~/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data"
        csv_path = os.path.expanduser(original_path) + '/' + AGENT + "_goal.csv"
        self.goal_csv = pd.read_csv(csv_path)
        
        # Person subscriber
        self.sub_people = rospy.Subscriber('/people_tracker/people', People, self.cb_handle_data)
        self.pub_selh = rospy.Publisher('/hri/sel_human', TrackedPersons, queue_size=10)
        
        
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
                                                    
                
    def cb_handle_data(self, people: People):
        """
        People callback

        Args:
            people (People): tracked people
        """
        
        for p in people.people:
            if int(p.name) in self.A_ID:
                
                pose = self.extract_pose(p)
                
                # check if traj point is contained in the map 
                if not MAP.contains(Point(pose.pose.position.x, pose.pose.position.y)):
                    # rospy.logerr(str(p.name) + " OUT")
                    continue
                 
                if int(p.name) != self.selHID:
                    self.old_pos = None
                    self.prev_time = None
                self.selHID = int(p.name)
                
                if self.old_pos is None or self.prev_time is None:
                    self.prev_time = people.header.stamp
                    self.old_pos = self.extract_pose(p)
                    return
                
                current_time = people.header.stamp

                # Calculate time difference
                delta_t = (current_time - self.prev_time).to_sec()

                # Calculate omega
                _, _, prev_yaw = euler_from_quaternion([self.old_pos.pose.orientation.x, self.old_pos.pose.orientation.y, self.old_pos.pose.orientation.z, self.old_pos.pose.orientation.w])
                _, _, current_yaw = euler_from_quaternion([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w])
                delta_yaw = current_yaw - prev_yaw
                omega = delta_yaw / delta_t

                # Update previous pose and time
                self.old_pos = pose
                self.prev_time = current_time
                
                twist = TwistWithCovariance()
                twist.twist.linear.x = p.velocity.x
                twist.twist.linear.y = p.velocity.y
                twist.twist.linear.z = p.velocity.z
                twist.twist.angular.x = 0
                twist.twist.angular.y = 0
                twist.twist.angular.z = omega
                twist.covariance = [0.0] * 36
                
                sh = TrackedPerson()
                # sh.track_id = int(p.name)
                sh.track_id = 1000
                sh.pose = pose
                sh.twist = twist
                
                msg = TrackedPersons()
                msg.header = Header()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.tracks.append(sh)
                self.pub_selh.publish(msg)
                
                row_index = self.goal_csv.index[self.goal_csv['time'] == people.header.stamp.to_sec()].tolist()
                if row_index:
                    row = self.goal_csv.loc[row_index[0]]
                    rospy.set_param(GOAL_PARAM, [float(row['h_{gx}']), float(row['h_{gy}'])])

                return
                  

if __name__ == '__main__':       
    AGENT = sys.argv[1]
    PEOPLE_ID = sys.argv[2]

    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    rospy.spin()