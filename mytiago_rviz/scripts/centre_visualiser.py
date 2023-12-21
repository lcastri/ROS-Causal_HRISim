#!/usr/bin/env python

import rospy
from visualization_msgs.msg import Marker

def publish_circle_marker():
    rospy.init_node('centre_marker_publisher', anonymous=True)
    marker_pub = rospy.Publisher('centre_marker', Marker, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    marker = Marker()
    marker.header.frame_id = "map"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.scale.x = 1.0  # Diameter of the circle
    marker.scale.y = 1.0  # Diameter of the circle
    marker.scale.z = 0.1  # Thickness of the circle
    marker.color.a = 1.0  # Alpha (transparency)
    marker.color.r = 1.0  # Red color
    marker.color.g = 0.0  # Green color
    marker.color.b = 0.0  # Blue color

    # Set the position of the circle
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0

    while not rospy.is_shutdown():
        marker.header.stamp = rospy.Time.now()
        marker_pub.publish(marker)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_circle_marker()
    except rospy.ROSInterruptException:
        pass
