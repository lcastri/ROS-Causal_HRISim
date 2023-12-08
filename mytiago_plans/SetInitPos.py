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


def SetInitialPos(p):
        p.exec_action('goto', "7.5_-7.5_0.0")
                      


if __name__ == "__main__":

    p = PNPCmd()

    p.begin()

    SetInitialPos(p)

    p.end()
