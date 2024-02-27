import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
from pnp_msgs.srv import PNPCondition, PNPConditionValue
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryActionGoal

class moveTorso(AbstractAction):

    def _start_action(self):
        
        rospy.loginfo('Torso movement action to position ' + " ".join(self.params) + ' ...')

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass vertical position to reach")
        else:
            self.ac = actionlib.SimpleActionClient("/torso_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            rospy.loginfo("Connecting to /torso_controller/follow_joint_trajectory AS...")
            self.ac.wait_for_server()
            rospy.loginfo("Connected.")

            # create goal
            self.goal = FollowJointTrajectoryActionGoal()
            # rospy.loginfo(self.goal)
            self.goal.goal.trajectory.joint_names = ['torso_lift_joint']
            pt = JointTrajectoryPoint()
            pt.positions = [float(self.params[0])]
            pt.time_from_start = rospy.Duration(2.0)
            self.goal.goal.trajectory.points.append(pt)

            # send goal
            self.ac.send_goal(self.goal.goal, done_cb=self._on_move_done)
            rospy.loginfo("Waiting for result...")
            

    def _on_move_done(self, goalState, result):
        print("Torso movement DONE", goalState, result)

        self.params.append("done")

    def _stop_action(self):

        self.ac.cancel_all_goals()
        rospy.loginfo('STOPPED move torso action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached
