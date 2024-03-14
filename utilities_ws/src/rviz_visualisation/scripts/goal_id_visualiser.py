#!/usr/bin/env python

import json
import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import sys
from people_msgs.msg import People
import shapely.geometry


MAP_BOUNDARIES = [(5.45, -4.66), (0.75, -0.56), (0.47, -0.73), (-0.73, 0.28), 
                  (0.01, 1.05), (-0.37, 1.37), (0.69, 2.62), (1.29, 2.14),
                  (2.01, 2.82), (2.95, 1.82), (3.52, 1.4), (4.16, 1.17), (7.86, -1.85)]
MAP = shapely.geometry.Polygon(MAP_BOUNDARIES)  


def cb_people(people: People):
    """
    People callback
    
    Args:
        people (People): tracked people
    """
    
    for p in people.people:
        if int(p.name) in PERSON_ID:
            # check if traj point is contained in the map 
            h = shapely.geometry.Point(p.position.x, p.position.y)
            if not MAP.contains(h):
                continue
                      
            # Create G1 Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.5  # Font size
            marker.color.a = 1.0  # Alpha (transparency)
            if h.distance(shapely.geometry.Point(float(GOALS["G1"]["x"]), float(GOALS["G1"]["y"]))) < float(GOALS["G1"]["radius"]):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            # Set the text content and position
            marker.text = "G1"
            marker.pose.position = Point(float(GOALS["G1"]["x"]), float(GOALS["G1"]["y"]), 0.0)
                    
            # Publish the RViz marker
            marker.header.stamp = rospy.Time.now()
            g1_pub.publish(marker)
                
                
                
            # Create G2 Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.5  # Font size
            marker.color.a = 1.0  # Alpha (transparency)
            if h.distance(shapely.geometry.Point(float(GOALS["G2"]["x"]), float(GOALS["G2"]["y"]))) < float(GOALS["G2"]["radius"]):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            # Set the text content and position
            marker.text = "G2"
            marker.pose.position = Point(float(GOALS["G2"]["x"]), float(GOALS["G2"]["y"]), 0.0)
                
            # Publish the RViz marker
            marker.header.stamp = rospy.Time.now()
            g2_pub.publish(marker)
            
            
            
            # Create G3 Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.5  # Font size
            marker.color.a = 1.0  # Alpha (transparency)
            if h.distance(shapely.geometry.Point(float(GOALS["G3"]["x"]), float(GOALS["G3"]["y"]))) < float(GOALS["G3"]["radius"]):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0

            # Set the text content and position
            marker.text = "G3"
            marker.pose.position = Point(float(GOALS["G3"]["x"]), float(GOALS["G3"]["y"]), 0.0)
                
            # Publish the RViz marker
            marker.header.stamp = rospy.Time.now()
            g3_pub.publish(marker)
            
            
            
            # Create G4 Marker
            marker = Marker()
            marker.header.frame_id = "map"
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.scale.z = 0.5  # Font size
            marker.color.a = 1.0  # Alpha (transparency)
            if h.distance(shapely.geometry.Point(float(GOALS["G4"]["x"]), float(GOALS["G4"]["y"]))) < float(GOALS["G4"]["radius"]):
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            else:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            
            # Set the text content and position
            marker.text = "G4"
            marker.pose.position = Point(float(GOALS["G4"]["x"]), float(GOALS["G4"]["y"]), 0.0)
                    
            # Publish the RViz marker
            marker.header.stamp = rospy.Time.now()
            g4_pub.publish(marker)


if __name__ == '__main__':
    AGENT = sys.argv[1]

    with open(sys.argv[2]) as json_file:
        GOALS = json.load(json_file)
        
    with open(sys.argv[3]) as json_file:
        peopleID = json.load(json_file)
        PERSON_ID = peopleID[AGENT]

    # Initialize ROS node
    rospy.init_node('goal_id_visualiser', anonymous=True)
    
    # Subscriber People
    rospy.Subscriber('/people_tracker/people', People, cb_people)

    # Publisher for RViz Marker
    g1_pub = rospy.Publisher('g1_marker', Marker, queue_size=1)
    g2_pub = rospy.Publisher('g2_marker', Marker, queue_size=1)
    g3_pub = rospy.Publisher('g3_marker', Marker, queue_size=1)
    g4_pub = rospy.Publisher('g4_marker', Marker, queue_size=1)

    rate = rospy.Rate(10) # Rate in Hz
    
    rospy.spin()