import os
import random
import subprocess
import sys

import rospy
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


def Thor(p):
    RECORD = False if str(sys.argv[1]) == '0' else True
    if RECORD:
        BAGNAME = str(sys.argv[2])
        BAGSEC = float(sys.argv[3]) # [s]
    else:
        NCYCLE = int(sys.argv[2])
        
    rospy.set_param('/hri/robot_goalreached', True)
    
    # Map inb3235_small
    R1 = [4.999, -1.003, -2.314]
    R2 = [4.065, -2.098, 2.462]          
    R3 = [1.659, -0.065, 0.862]          
    R4 = [2.403, 1.029, -0.686]         
    Rgoals = [R1, R2, R3, R4]
    
    # Map inb3235_new
    # R1 = [4.527, -0.213, -1.497]
    # R2 = [4.660, -3.145, -1.571]
    # R3 = [2.385, -3.418, 3.025]
    # R4 = [2.568, -0.288, 1.571]
    # Rgoals = [R1, R2, R3, R4]
    
    # R5 = [0.630, 4.515, -1.566]
    # R6 = [0.630, 0.248, 1.566]
    # Rgoals = [R5, R6]
    
    p.exec_action('moveHead', '0.0_-0.83')
    p.exec_action("moveTorso", '0.1')
    p.exec_action("moveArm", "home")
    p.exec_action("moveTorso", '0.02')
    
    if RECORD:
        # Start recording
        p.action_cmd('record', BAGNAME, 'start')
        # this is to make sure that the recording has started
        while not HasRecordingStarted():
            time.sleep(.1)
        start_recording = rospy.Time.now()
        time.sleep(1)
    
        while (rospy.Time.now() - start_recording).to_sec() <= BAGSEC:           
            for rg in Rgoals:
                if rospy.get_param('/hri/robot_goalreached'):
                    rospy.set_param('/hri/robot_goalreached', False)
                    p.exec_action('goto', '_'.join([str(coord) for coord in rg]))

        # Stop recording
        p.action_cmd('record', BAGNAME, 'stop')
        # this is to make sure that the recording i-th is stopped
        while not HasRecordingStopped():
            time.sleep(.1)
            
    else:
        for i in range(NCYCLE):           
            for rg in Rgoals:
                if rospy.get_param('/hri/robot_goalreached'):
                    rospy.set_param('/hri/robot_goalreached', False)
                    p.exec_action('goto', '_'.join([str(coord) for coord in rg]))



if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Thor(p)

    p.end()