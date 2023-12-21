import os
import sys
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import pnp_cmd_ros
from pnp_cmd_ros import *
import time

def PassByTheCentre(p):
    seconds = 45
    p.exec_action('goto', "-5_5_-0.739")
    
    for i in range(seconds, 0, -1):
        print("Countdown: " + str(i))
        time.sleep(1)  # Wait for 1 second
    
    goal_list = [["5", "-5", "2.347"], 
                 ["-5","5", "-0.739"]]
    for _ in range(15):
        for g in goal_list:
            p.exec_action('goto', "_".join(g))
                      


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PassByTheCentre(p)

    p.end()
