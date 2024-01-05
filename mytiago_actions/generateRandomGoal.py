import os
import random
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

from AbstractAction import AbstractAction
import rospy
# from visualization_msgs.msg import Marker
# from geometry_msgs.msg import Point


def generate_random_point(p1, p2, p3):
    """
    Creates a random point within a certain area specified by three points

    Args:
        p1 (list): point
        p2 (list): point
        p3 (list): point

    Returns:
        list: random point
    """
    x, y = random.random(), random.random()
    q = abs(x - y)
    s, t, u = q, 0.5 * (x + y - q), 1 - 0.5 * (q + x + y)
    return [
        s * p1[0] + t * p2[0] + u * p3[0],
        s * p1[1] + t * p2[1] + u * p3[1],
    ]


class generateRandomGoal(AbstractAction):

    def _start_action(self):
        
        #NOTE: Assume self.params is a list of strings the first element is the name of the node to navigate to
        rospy.loginfo('Random goal area ' + " ".join(self.params))
        p1 = [float(self.params[0]), float(self.params[1])]
        p2 = [float(self.params[2]), float(self.params[3])]
        p3 = [float(self.params[4]), float(self.params[5])]

        if len(self.params) < 1:
            rospy.logwarn("Wrong use of action, pass the triangle coordinates where to generate the random point")
        else:
           
            g = generate_random_point(p1, p2, p3)
            
            print("GENERATED POINT: " + str(g))
            rospy.set_param('/ped/random_goal', g)

            self.params.append("done")
    
            
    def _stop_action(self):
        self.params.append("interrupted")
        rospy.loginfo('STOPPED generateRandomGoal action')
    
    
    @classmethod
    def is_goal_reached(cls, params):
        reached = False
        if len(params) > 0 and params[-1] == "done":
            reached = True
        return reached