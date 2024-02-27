#!/usr/bin/env python

import math
import os

from geometry_msgs.msg import PoseWithCovariance, TwistWithCovariance
import rospy
import pandas as pd
from people_msgs.msg import People
from pedsim_msgs.msg import TrackedPersons, TrackedPerson
import tf
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler, euler_from_quaternion

NODE_NAME = 'extract_sel_human'
NODE_RATE = 10 #Hz
            

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
        p = people.people[0]
            
        if self.old_pos is None or self.prev_time is None:
            self.prev_time = people.header.stamp
            self.old_pos = self.extract_pose(p)
            return
        
        current_time = people.header.stamp
        pose = self.extract_pose(p)

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
        sh.track_id = 1000
        sh.pose = pose
        sh.twist = twist
        
        msg = TrackedPersons()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.tracks.append(sh)
        self.pub_selh.publish(msg)
                  

if __name__ == '__main__':        
    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
   
    data_handler = DataHandler()
        
    rospy.spin()