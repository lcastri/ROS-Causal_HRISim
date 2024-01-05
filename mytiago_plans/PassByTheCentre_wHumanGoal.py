import os
import subprocess
import sys
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
import time

def findRecordingNode(prefix = "/record_"):
    command = ['rosnode', 'list']  # Command to list all active ROS nodes
    process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    stdout, stderr = process.communicate()
    matching_nodes = list()

    if process.returncode == 0:

        # Split the output by newline to get a list of node names
        active_nodes = stdout.splitlines()

        # Filter the node names that start with the specified prefix
        matching_nodes = [node for node in active_nodes if node.startswith(prefix)]
    return matching_nodes


def HasRecordingStopped():
    recordingNode = findRecordingNode()
    if len(recordingNode) > 0:
        return False
    else:
        return True
    
    
def HasRecordingStarted():
    recordingNode = findRecordingNode()
    if len(recordingNode) > 0:
        return True
    else:
        return False


def PassByTheCentre(p):
    # SECONDS = 30
    N = int(sys.argv[1])
    BAGNAME = str(sys.argv[2])
    G1 = ["5", "-5", "-0.739"]
    G1bis = ["5", "-5", "2.347"]
    G2 = ["-5","5", "2.347"]
    G2bis = ["-5","5", "-0.739"]
    A1 = ["3.5", "-5", "-5", "3.5", "-6.5", "-6.5"]
    A2 = ["5", "-3.5", "-3.5", "5", "6.5", "6.5"]
    
    p.exec_action('goto', "-5_5_-0.739")
       
    goal_list = [G1, G2]
    goalbis_list = [G1bis, G2bis]
    area_list = [A1, A2]
    
    count = 0
    for _ in range(N):
        for g, gbis, a in zip(goal_list, goalbis_list, area_list):
            
            # Start recording
            p.action_cmd('record', BAGNAME + str(count), 'start')
            # this is to make sure that the recording has started
            # before executing run i-th
            while not HasRecordingStarted():
                time.sleep(.1)
            time.sleep(1)
                
            # Go to goal
            p.exec_action('generateRandomGoal', "_".join(a))
            p.exec_action('goto', "_".join(g))
            
            # Stop recording
            p.action_cmd('record', BAGNAME + str(count), 'stop')
            # this is to make sure that the recording i-th is stopped 
            # before executing run i+1 th
            while not HasRecordingStopped():
                time.sleep(.1)
                
            # Turn around
            p.exec_action('goto', "_".join(gbis))
            # Increase rosbag number
            count += 1
            # Sleep before restarting
            time.sleep(1)


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PassByTheCentre(p)

    p.end()