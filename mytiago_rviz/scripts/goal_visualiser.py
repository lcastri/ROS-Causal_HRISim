#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

def get_goal_from_rosparam():
    # Initialize ROS node
    rospy.init_node('goal_visualiser', anonymous=True)
    
    # Publisher for RViz Marker
    marker_pub = rospy.Publisher('goal_marker', Marker, queue_size=10)

    rate = rospy.Rate(1)  # Rate in Hz
    
    while not rospy.is_shutdown():
        try:
            # Read goal points from rosparam
            g = rospy.get_param('/hri/human_goal')
        except:
            continue
        
        # Create RViz Marker
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 1.0  # Diameter of the circle
        marker.scale.y = 1.0  # Diameter of the circle
        marker.scale.z = 0.1  # Thickness of the circle
        marker.color.a = 1.0  # Alpha (transparency)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # Set the goal points as the marker position
        marker.pose.position = Point(x = g[0], y = g[1], z = 0.0)

        # Publish the RViz marker
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        get_goal_from_rosparam()
    except rospy.ROSInterruptException:
        pass
