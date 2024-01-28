#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from mytiago_human.msg import HumanState, Risk

def risk_callback(human):

    line_marker = Marker()
    line_marker.header.frame_id = "map"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.type = Marker.LINE_LIST
    line_marker.action = Marker.ADD
    line_marker.pose.orientation.w = 1.0
    line_marker.scale.x = 0.05  # Set the line width

    if human.risk.collision.data:
        line_marker.color.r = 1.0 
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
    else:
        line_marker.color.r = 0.0 
        line_marker.color.g = 0.75
        line_marker.color.b = 0.0
    line_marker.color.a = 1.0

    # Add lines connecting the points
    line_marker.points.append(human.risk.origin)
    line_marker.points.append(human.risk.left)

    line_marker.points.append(human.risk.origin)
    line_marker.points.append(human.risk.right)

    line_marker.points.append(human.risk.right)
    line_marker.points.append(human.risk.left)

    triangle_pub.publish(line_marker)

    

if __name__ == '__main__':
    try:
        rospy.init_node('risk_publisher', anonymous=True)
        triangle_pub = rospy.Publisher('risk_marker', Marker, queue_size=10)
        risk_sub = rospy.Subscriber('/hri/human_state', HumanState, risk_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass