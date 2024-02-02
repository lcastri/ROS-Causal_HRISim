#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32


class TwistMuxNode:
    def __init__(self):
        rospy.init_node('mytwist_mux')
        
        # Initialize scaling factor and previous tmp_cmd_vel
        self.scaling_factor = 1.0

        # Subscribe to twist_mux/tmp_cmd_vel and twist_mux/scaling_factor topics
        rospy.Subscriber('mobile_base_controller/tmp_cmd_vel', Twist, self.tmp_cmd_vel_callback)
        rospy.Subscriber('twist_mux/scaling_factor', Float32, self.scaling_factor_callback)
        
        # Publisher for twist_mux/cmd_vel
        self.cmd_vel_pub = rospy.Publisher('mobile_base_controller/cmd_vel', Twist, queue_size=10)

    def tmp_cmd_vel_callback(self, msg:Twist):
        tmp_cmd_vel = msg
        
        # Calculate the derived cmd_vel
        cmd_vel = Twist()
        cmd_vel.linear.x = tmp_cmd_vel.linear.x * self.scaling_factor
        cmd_vel.linear.y = tmp_cmd_vel.linear.y * self.scaling_factor
        cmd_vel.linear.z = tmp_cmd_vel.linear.z * self.scaling_factor
        cmd_vel.angular.x = tmp_cmd_vel.angular.x * self.scaling_factor
        cmd_vel.angular.y = tmp_cmd_vel.angular.y * self.scaling_factor
        cmd_vel.angular.z = tmp_cmd_vel.angular.z * self.scaling_factor

        # Publish the derived cmd_vel
        self.cmd_vel_pub.publish(cmd_vel)

    def scaling_factor_callback(self, msg):
        self.scaling_factor = msg.data

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        twist_mux_node = TwistMuxNode()
        twist_mux_node.run()
    except rospy.ROSInterruptException:
        pass
