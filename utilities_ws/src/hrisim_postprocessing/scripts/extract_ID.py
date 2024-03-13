#!/usr/bin/env python

import math
import os
import sys

import rospy
import pandas as pd
from people_msgs.msg import People
from shapely.geometry import Polygon, Point


NODE_NAME = 'extract_ID'
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
        self.people_list = list()
        columns = ['time', 'h', 'h_v']
        self.IDs = pd.DataFrame(columns=columns)
        
        rospy.Subscriber('/people_tracker/people', People, self.cb_filter_people)

           
    def cb_filter_people(self, people: People):
        for p in people.people:
            # rospy.logwarn("ID: " + p.name)
            # check if traj point is contained in the map 
            if not MAP.contains(Point(p.position.x, p.position.y)): continue
            # rospy.logwarn("inMAP: " + p.name)
            
            v = math.sqrt(p.velocity.x**2 + p.velocity.y**2)
            if v >= 0.15 and v <= 2:
                rospy.logwarn("ID: " + p.name)
                
                self.IDs.loc[len(self.IDs)] = {'time': people.header.stamp.to_sec(), 
                                               'h': p.name,
                                               'h_v': v}
                if int(p.name) not in self.people_list:
                    self.people_list.append(int(p.name))
                

if __name__ == '__main__':       
    
    csv_name = sys.argv[1]

    # Init node
    rospy.init_node(NODE_NAME)
    
    # Set node rate
    rate = rospy.Rate(NODE_RATE)
    
    def cleanup():
        original_path = "~/git/ROS-Causal_HRISim/utilities_ws/src/bag_postprocess_bringup/data"
        csv_path = os.path.expanduser(original_path)
        
        fIDs = data_handler.IDs[data_handler.IDs['h_v'] > 1]
        h_values_greater_than_1 = fIDs['h'].unique()
        result_df = data_handler.IDs[data_handler.IDs['h'].isin(h_values_greater_than_1)]
        
        data_handler.IDs.to_csv(csv_path + '/' + csv_name + "_IDs.csv", sep=',', index=False)
        result_df.to_csv(csv_path + '/' + csv_name + "_fIDs.csv", sep=',', index=False)
        rospy.logerr("IDs: " + str(data_handler.people_list))

    rospy.on_shutdown(cleanup)
   
    data_handler = DataHandler()
        
    rospy.spin()