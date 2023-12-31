import os
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import rospy
from AbstractAction import AbstractAction
import subprocess
import signal

"""
Starts and stops the object detection node
"""
class record(AbstractAction):
    
    def find_rosnode_with_prefix(self, prefix):
        command = ['rosnode', 'list']  # Command to list all active ROS nodes
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
        stdout, stderr = process.communicate()

        if process.returncode == 0:

            # Split the output by newline to get a list of node names
            active_nodes = stdout.splitlines()

            # Filter the node names that start with the specified prefix
            matching_nodes = [node for node in active_nodes if node.startswith(prefix)]

            if len(matching_nodes) > 0:
                return matching_nodes[0]
            else:
                return -1
        else:
            print(f"Error: {stderr}")
            return None

    def _start_action(self):
        rospy.loginfo('STARTED record action')
        if self.params:
           
            command = "rosbag record -O /root/shared/" + str(self.params[0]) + ".bag /map /mobile_base_controller/odom /move_base/goal /ped/control/teleop_persons /pedsim_simulator/simulated_agents /robot_pose /tf /tf_static"
            
            # Start recording.
            self.process = subprocess.Popen(command, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')

        else:
            rospy.logwarn("Wrong use of action, pass the rosbag name")  

    def _stop_action(self):
        pname = self.find_rosnode_with_prefix("/record_")
        # rospy.logerr("Killing node: " + pname)
        subprocess.Popen("rosnode kill " + pname, stdin=subprocess.PIPE, shell=True, executable='/bin/bash')
        self.params.append("interrupted")
        rospy.loginfo('STOPPED record action')

            
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached
