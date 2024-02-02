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

from pedsim_msgs.msg import TrackedPersons
from AbstractTopicCondition import AbstractTopicCondition

DIST_THRESH = 0.5 # [m]

class HasHumanArrived(AbstractTopicCondition):
    _topic_name = "/ped/control/teleop_persons"
    _topic_type = TrackedPersons

    def _get_value_from_data(self, data: TrackedPersons):
        return [data.tracks[0].pose.pose.position.x, data.tracks[0].pose.pose.position.y]
            
    def evaluate(self, params):
        # Read goal points from rosparam
        g = rospy.get_param('/hri/human_goal', None)
        
        if g is not None and math.dist(self.last_value, g) <= DIST_THRESH:
            self.last_value = None
            return True
        return False