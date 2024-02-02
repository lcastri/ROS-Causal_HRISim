import os
from shapely.geometry import Polygon, Point
import random
import sys

try:
    sys.path.insert(0, os.environ["PNP_HOME"] + '/actions')
except:
    print("Please set PNP_HOME environment variable to PetriNetPlans folder.")
    sys.exit(1)

from AbstractAction import AbstractAction
import rospy


class randomPointFromPolygon(AbstractAction):

    def _start_action(self):
        
        rospy.loginfo('Random goal area ' + " ".join(self.params))
        points = [float(p) for p in self.params]
        
        if len(points) < 6 or len(points) % 2 != 0:
            raise ValueError("The list must have at least 3 pairs of coordinates to form a polygon")

        # Split the list into pairs of x and y coordinates
        coordinates = [(points[i], points[i + 1]) for i in range(0, len(points), 2)]
        

        # Create a polygon from the coordinates
        polygon = Polygon(coordinates)

        # Generate a random point within the polygon
        min_x, min_y, max_x, max_y = polygon.bounds
        rp = Point(random.uniform(min_x, max_x), random.uniform(min_y, max_y))

        # Check if the point is within the polygon, regenerate if not
        while not polygon.contains(rp):
            rp = Point(random.uniform(min_x, max_x), random.uniform(min_y, max_y))

        rospy.set_param('/hri/human_goal', [float(rp.x), float(rp.y)])

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