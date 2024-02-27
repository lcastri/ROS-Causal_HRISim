import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import actionlib
from control_msgs.msg import FollowJointTrajectoryActionGoal, FollowJointTrajectoryAction
from trajectory_msgs.msg import JointTrajectoryPoint
"""
Move the head to a specific pan-tilt angle configuration.
Arguments:
    - pan angle
    - tile angle
"""
class moveHead(AbstractAction):

    def _start_action(self):
        rospy.loginfo('Head movement action to position ' + " ".join(self.params) + ' ...')
        #self.starting_time = rospy.Time.now()

        if len(self.params) < 2:
            rospy.logwarn("Wrong use of action, pass pan and tilt angles as arguments")
        else:
            self.ac = actionlib.SimpleActionClient("/head_controller/follow_joint_trajectory", FollowJointTrajectoryAction)
            rospy.loginfo("Connecting to /head_controller/follow_joint_trajectory AS...")
            self.ac.wait_for_server()
            rospy.loginfo("Connected.")

            # create goal
            self.goal = FollowJointTrajectoryActionGoal()
            # rospy.loginfo(self.goal)
            self.goal.goal.trajectory.joint_names = ['head_1_joint', 'head_2_joint']
            pt = JointTrajectoryPoint()
            pt.positions = [float(self.params[0]), float(self.params[1])]
            pt.time_from_start = rospy.Duration(2.0)
            self.goal.goal.trajectory.points.append(pt)

            # send goal
            self.ac.send_goal(self.goal.goal, done_cb=self._on_move_done)
            rospy.loginfo("Waiting for result...")

    def _on_move_done(self, goalState, result):
        print("Head movement DONE", goalState, result)

        self.params.append("done")

    def _stop_action(self):

        self.ac.cancel_all_goals()
        rospy.loginfo('STOPPED move head action')

    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True

        return reached