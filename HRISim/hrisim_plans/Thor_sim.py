import os
import random
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



def Thor_sim(p):
    H1 = ["5", "-5", "5", "-9", "9", "-5", "7.5", "-8"]
    H2 = ["7.5", "4.5", "6", "4.5", "6", "3", "7.5", "3"]
    H3 = ["-7.5", "7", "-5.5", "7", "-5.5", "9", "-7.5", "9"]
    H4 = ["-8", "-5.5", "-10", "-5.5", "-10", "-2.5", "-8", "-2.5"]
    R1 = ["2", "-5", "0"]
    R2 = ["2", "5", "1.57"]
    R3 = ["-2", "5", "3.14"]
    R4 = ["-2", "-5", "-1.57"]
    
    old_area = None
    Hareas = [H1, H2, H3, H4]
    noHgoal = True
    
       
    Rgoals = [R1, R2, R3, R4]
    # noRgoal = True
    rospy.set_param('/hri/robot_goalreached', True)
    g = -1
    
    
    while True:
        
        if noHgoal or p.get_condition("HasHumanArrived"):
            # Human goal
            a = random.choice(Hareas)
            while old_area is not None and a == old_area:
                a = random.choice(Hareas)
            old_area = a
                
            p.exec_action('randomPointFromPolygon', "_".join(a))
            noHgoal = False
            
            
        
        # if noRgoal or p.get_condition("HasRobotArrived"):
        if rospy.get_param('/hri/robot_goalreached'):
            rospy.set_param('/hri/robot_goalreached', False)
            # if not noRgoal: p.action_cmd('goto', "_".join(Rgoals[g]), 'stop')
            # Robot goal
            if g + 1 > len(Rgoals) - 1:
                g = 0
            else:
                g += 1
            p.action_cmd('goto', "_".join(Rgoals[g]), 'start')
            # noRgoal = False
    

if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    Thor_sim(p)

    p.end()