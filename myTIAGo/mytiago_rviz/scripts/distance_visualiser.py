#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker
from pedsim_msgs.msg import TrackedPersons
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from message_filters import ApproximateTimeSynchronizer, Subscriber


def define_color(distance, marker):
    if distance > DIST_THRESHOLD:
        marker.color.b = 1.0
        marker.color.r = 0.0
    else:
        marker.color.b = 0.0
        marker.color.r = 1.0
        

def distance_callback(person, robot):
    person_point = person.tracks[0].pose.pose.position
    person_point.z = 0
    robot_point = robot.pose.pose.position
    robot_point.z = 0
        
    # Calculate distance between the two points
    distance = ((robot_point.x - person_point.x)**2 + (robot_point.y - person_point.y)**2)**0.5

    # Create a Marker message to display the distance as text
    text_marker = Marker()
    text_marker.header.frame_id = "map" 
    text_marker.header.stamp = rospy.Time.now()
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD
    text_marker.scale.z = 0.3  # Text size
    text_marker.pose.position = person_point
    text_marker.pose.position.z += 2  # Offset text slightly above point1
    text_marker.text = "dist: {:.2f}m".format(distance)
    define_color(distance, text_marker)
    text_marker.color.a = 1.0

    line_marker = Marker()
    line_marker.header.frame_id = "map"
    line_marker.header.stamp = rospy.Time.now()
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.05
    line_marker.points.append(Point(x = person_point.x, y = person_point.y, z = 0))
    line_marker.points.append(Point(x = robot_point.x, y = robot_point.y, z = 0))
    define_color(distance, line_marker)
    line_marker.color.a = 1.0

    text_pub.publish(text_marker)
    line_pub.publish(line_marker)


if __name__ == '__main__':
    rospy.init_node('distance_marker_publisher')
    
    DIST_THRESHOLD = rospy.get_param('/hri/safe_distance', 5) 

    person_sub = Subscriber('/ped/control/teleop_persons', TrackedPersons)
    robot_sub = Subscriber('/robot_pose', PoseWithCovarianceStamped)
    synchronizer = ApproximateTimeSynchronizer([person_sub, robot_sub], queue_size=10, slop=0.1)
    synchronizer.registerCallback(distance_callback)

    text_pub = rospy.Publisher('distance_text_marker', Marker, queue_size=10)
    line_pub = rospy.Publisher('distance_line_marker', Marker, queue_size=10)

    rospy.spin()
