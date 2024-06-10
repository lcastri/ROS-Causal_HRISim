#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from roscausal_msgs.msg import Humans

def person_callback(people):
    try:
        SELAGENT_ID = int(rospy.get_param("/hri/selected_agent_id"))
        for person in people.humans:
            if person.id == SELAGENT_ID:
                # LINE       
                line_marker = Marker()
                line_marker.header.frame_id = "map"
                line_marker.header.stamp = rospy.Time.now()
                line_marker.type = Marker.LINE_STRIP
                line_marker.action = Marker.ADD
                line_marker.scale.x = 0.05
                line_marker.points.append(Point(x = person.pose2D.x, y = person.pose2D.y, z = 0))
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
    person_sub = rospy.Subscriber('/roscausal/humans', Humans, person_callback)
    rate = rospy.Rate(1)  # Rate in Hz

    while not rospy.is_shutdown():
        try:
            # Read goal points from rosparam
            GOAL = rospy.get_param('/hri/human_goal')
            rate.sleep()
        except:
            continue
