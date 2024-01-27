#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from pedsim_msgs.msg import TrackedPersons

def linegoal_callback(person):
    try:
        person_point = person.tracks[0].pose.pose.position
        person_point.z = 0
        
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = rospy.Time.now()
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.05
        line_marker.points.append(Point(x = person_point.x, y = person_point.y, z = 0))
        line_marker.points.append(Point(x = GOAL[0], y = GOAL[1], z = 0))
        line_marker.color.r = 0.0
        line_marker.color.g = 0.0
        line_marker.color.b = 1.0
        line_marker.color.a = 1.0

        line_pub.publish(line_marker)
    except:
        pass

    

if __name__ == '__main__':

    rospy.init_node('linegoal_publisher', anonymous=True)
    line_pub = rospy.Publisher('linegoal_marker', Marker, queue_size=10)
    person_sub = rospy.Subscriber('/ped/control/teleop_persons', TrackedPersons, linegoal_callback)
    rate = rospy.Rate(1)  # Rate in Hz

    while not rospy.is_shutdown():
        try:
            # Read goal points from rosparam
            GOAL = rospy.get_param('/hri/human_goal')
            g = rospy.get_param('/hri/human_goal')
            rate.sleep()
        except:
            continue
