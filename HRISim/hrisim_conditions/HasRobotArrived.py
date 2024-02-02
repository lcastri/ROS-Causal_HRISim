import math
import os
import sys
import rospy
try:
    rospy.logwarn("asdsd")
    sys.path.insert(0, os.environ["PNP_HOME"] + '/conditions')
    rospy.logwarn("sys path inserted")
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    rospy.logwarn("Error inserting sys path")
    sys.exit(1)

from nav_msgs.msg import Odometry
from AbstractTopicCondition import AbstractTopicCondition

DIST_THRESH = 1.0 # [m]

class HasRobotArrived(AbstractTopicCondition):
    _topic_name = "/mobile_base_controller/odom"
    _topic_type = Odometry

    def _get_value_from_data(self, p: Odometry):
        return [p.pose.pose.position.x, p.pose.pose.position.y]
            
    def evaluate(self, params):
        # Read goal points from rosparam
        g = rospy.get_param('/hri/robot_goal', None)
        
        if g is not None and math.dist(self.last_value, g) <= DIST_THRESH:
            self.last_value = None
            return True
        return False