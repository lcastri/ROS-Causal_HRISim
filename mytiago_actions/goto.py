import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import math


class goto(AbstractAction):

    def _start_action(self):
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Going to ' + " ".join(self.params) + ' ...')

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the coordinates the robots needs to reach in /map frame as X_Y_Theta")
        else:
            self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
            rospy.loginfo("Connecting to /move_base AS...")
            self.client.wait_for_server()
            rospy.loginfo("Connected.")

            # Create goal
            self.goal_msg = MoveBaseGoal()
            self.goal_msg.target_pose.header.frame_id = "map"
            self.goal_msg.target_pose.header.stamp = rospy.Time.now()
            self.goal_msg.target_pose.pose.position.x = float(self.params[0])
            self.goal_msg.target_pose.pose.position.y = float(self.params[1])
            self.goal_msg.target_pose.pose.orientation.z = math.sin(float(self.params[2]) / 2)
            self.goal_msg.target_pose.pose.orientation.w = math.cos(float(self.params[2]) / 2)

            self.client.send_goal(self.goal_msg, done_cb=self._on_goTo_done)
            rospy.loginfo("Waiting for goTo result...")

    def _on_goTo_done(self, goalState, result):
        print("goTo DONE", goalState, result)
        self.params.append("done")
        rospy.loginfo('Destination reached')

    def _stop_action(self):
        self.client.cancel_all_goals()
        self.params.append("interrupted")
        rospy.loginfo('STOPPED goto action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO: also make the necessary changes to make sure this returns True when the navigation has reached the goal

        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
