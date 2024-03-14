#!/usr/bin/env python

import pandas as pd
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from people_msgs.msg import People
import sys

def cb_people(people: People):
    """
    People callback
    
    Args:
        people (People): tracked people
    """
                      
    row_index = GOAL_CSV.index[GOAL_CSV['time'] == people.header.stamp.to_sec()].tolist()
    if row_index:
        row = GOAL_CSV.loc[row_index[0]]
        
        # Create RViz Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = .75  # Diameter of the circle
        marker.scale.y = .75  # Diameter of the circle
        marker.scale.z = 0.1  # Thickness of the circle
        marker.color.a = 1  # Alpha (transparency)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Set the goal points as the marker position
        marker.pose.position = Point(x = float(row['h_{gx}']), y = float(row['h_{gy}']), z = 0.0)

        # Publish the RViz marker
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)


if __name__ == '__main__':
    
    GOAL_PATHCSV = sys.argv[1]
    GOAL_CSV = pd.read_csv(sys.argv[1])

    # Initialize ROS node
    rospy.init_node('goal_dot_visualiser', anonymous=True)
        
    # Subscriber People
    rospy.Subscriber('/people_tracker/people', People, cb_people)

    # Publisher for RViz Marker
    marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)

    rate = rospy.Rate(10) # Rate in Hz
    
    rospy.spin()
        
