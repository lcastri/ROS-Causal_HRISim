import math
import os
import sys
try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/scripts')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

import time
import pnp_cmd_ros
from pnp_cmd_ros import *


def PassByTheCentre(p):
    goal_list = [["5", "-5", "2.347"], 
                 ["-5","5", "-0.739"]]
    for _ in range(50):
        for g in goal_list:
            p.exec_action('goto', "_".join(g))
                      


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    PassByTheCentre(p)

    p.end()
