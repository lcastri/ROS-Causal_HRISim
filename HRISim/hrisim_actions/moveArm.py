import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
import actionlib
from AbstractAction import AbstractAction
from play_motion_msgs.msg import PlayMotionAction, PlayMotionActionGoal

"""
Starts and stops the object detection node
"""
class moveArm(AbstractAction):

    def _start_action(self):
        
        rospy.loginfo('Starting to ' + " ".join(self.params) + ' arm ...')
        
        self.ac = actionlib.SimpleActionClient("/play_motion", PlayMotionAction)
        rospy.loginfo("Connecting to /play_motion AS...")
        self.ac.wait_for_server()
        rospy.loginfo("Connected.")

        
        self.goal = PlayMotionActionGoal()
        
        #here put the code necessary to open/close the gripper
        self.goal.goal.motion_name = self.params[0]

        # send goal
        self.ac.send_goal(self.goal.goal, done_cb=self._on_motion_done)
        rospy.loginfo("Waiting for result...")

    def _on_motion_done(self, goalState, result):
        print("arm motion DONE", goalState, result)
        #rospy.Time.sleep(0)
        self.params.append("done")

    def _stop_action(self):
        #TODO: here put the code necessary to cleanly stop the gripper action

        self.ac.cancel_all_goals()
        self.params.append("done")
        rospy.loginfo('STOPPED move arm action')

    @classmethod
    def is_goal_reached(cls, params):
        #TODO make sure the below returns True when the gripper action has been execyted fully
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
