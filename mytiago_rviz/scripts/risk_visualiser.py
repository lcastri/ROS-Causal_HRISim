#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from mytiago_risk.msg import Risk
from message_filters import ApproximateTimeSynchronizer, Subscriber
from pedsim_msgs.msg import TrackedPersons

def risk_callback(risk, person):
    person_point = person.tracks[0].pose.pose.position
    
    background_marker = Marker()
    background_marker.header.frame_id = "map"
    background_marker.header.stamp = rospy.Time.now()
    background_marker.type = Marker.CUBE
    background_marker.action = Marker.ADD
    background_marker.pose.orientation.w = 1.0
    background_marker.scale.x = 1.0  
    background_marker.scale.y = 1.0
    background_marker.scale.z = 0.01  
    background_marker.pose.position = person_point 
    background_marker.pose.position.z += 2.0

    if risk.collision.data:
        background_marker.color.r = 1.0 
        background_marker.color.g = 0.0
        background_marker.color.b = 0.0
    else:
        background_marker.color.r = 0.0 
        background_marker.color.g = 1.0
        background_marker.color.b = 0.0
    background_marker.color.a = 0.75
    
    
    triangle_marker = Marker()
    triangle_marker.header.frame_id = "map"
    triangle_marker.header.stamp = rospy.Time.now()
    triangle_marker.type = Marker.TRIANGLE_LIST
    triangle_marker.action = Marker.ADD
    triangle_marker.pose.orientation.w = 1.0
    triangle_marker.scale.x = 1.0
    triangle_marker.scale.y = 1.0
    triangle_marker.scale.z = 1.0
    if risk.collision.data:
        triangle_marker.color.r = 1.0 
        triangle_marker.color.g = 0.0
    else:
        triangle_marker.color.r = 0.0 
        triangle_marker.color.g = 1.0
    triangle_marker.color.b = 0.0 
    triangle_marker.color.a = 0.5

    point1 = risk.origin
    point2 = risk.left
    point3 = risk.right

    triangle_marker.points.append(point1)
    triangle_marker.points.append(point2)
    triangle_marker.points.append(point3)

    triangle_pub.publish(triangle_marker)
    cube_pub.publish(background_marker)
    

if __name__ == '__main__':
    try:
        rospy.init_node('risk_publisher', anonymous=True)
        triangle_pub = rospy.Publisher('risk_marker', Marker, queue_size=10)
        cube_pub = rospy.Publisher('riskcube_marker', Marker, queue_size=10)
        risk_sub = Subscriber('/hri/risk', Risk)
        person_sub = Subscriber('/ped/control/teleop_persons', TrackedPersons)
        synchronizer = ApproximateTimeSynchronizer([risk_sub, person_sub], queue_size=10, slop=0.1)
        synchronizer.registerCallback(risk_callback)
        
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
