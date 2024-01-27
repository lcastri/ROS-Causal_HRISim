#!/usr/bin/env python

import rospy
from shapely.geometry import *
import math
import message_filters
from pedsim_msgs.msg import TrackedPersons
from mytiago_risk.msg import Risk
from mytiago_robot.msg import RobotState
from std_msgs.msg import Header


NODE_NAME = "mytiago_risk"
NODE_RATE = 10 # [Hz]
DIST_THRES = float(rospy.get_param("/hri/safe_distance", default = 5.0))
INC_SIZE = float(rospy.get_param("/hri/obs_size", default = 2.5))


def compute_risk(A: Point, obs: Point, Av: Point, obsv: Point):
    risk = 0
    collision = False
    
    # Calculate relative velocity vector
    Vrel = Point(obsv.x - Av.x, obsv.y - Av.y)
           
    # Compute the slope of the line AB and line PAB ⊥ AB
    slope_AB = (obs.y - A.y) / (obs.x - A.x)  # Rise over run
    slope_PAB = -1 / slope_AB
    # Choose distances to the left and right of point B

    # Calculate coordinates for two points along the perpendicular line
    # Calculate the change in x and y based on the fixed horizontal distance
    delta_x = INC_SIZE / (1 + slope_PAB ** 2) ** 0.5
    delta_y = delta_x * slope_PAB
    # Calculate coordinates for two points along the perpendicular line
    left = Point(obs.x - delta_x, obs.y - delta_y)
    right = Point(obs.x + delta_x, obs.y + delta_y)
    # Cone
    cone_origin = Point(A.x, A.y)               
    cone = Polygon([cone_origin, left, right])
    
    P = Point(cone_origin.x + Av.x, cone_origin.y + Av.y)
    collision = P.within(cone) and A.distance(obs) < DIST_THRES
    if collision:
        time_collision_measure = A.distance(obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
        steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
        risk = risk + 1/time_collision_measure + steering_effort_measure
                
    # return math.exp(risk)
    return risk, collision, cone_origin, left, right


class RiskClass():
    
    def __init__(self) -> None:
        """
        RiskClass constructor
        """

        # Risk publisher     
        self.pub_risk = rospy.Publisher('/hri/risk', Risk, queue_size=10)
        
        # Robot subscriber
        sub_robot = message_filters.Subscriber("/hri/robot_state", RobotState)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/ped/control/teleop_persons', TrackedPersons)
                
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.get_data)
                       

    def get_data(self, robot: RobotState, people: TrackedPersons):
        """
        Synchronized callback

        Args:
            robot (RobotState): robot state
            people (TrackedPersons): tracked people
        """
        # Robot 2D pose (x, y, theta) and velocity
        r_x = robot.pose2D.x
        r_y = robot.pose2D.y
        r_theta = robot.pose2D.theta
        r_v = robot.twist.linear

        # Human 2D pose (x, y, theta) and velocity
        p = people.tracks[0]            
        h_x = p.pose.pose.position.x
        h_y = p.pose.pose.position.y
        h_theta = p.pose.pose.orientation.z
        h_v = p.twist.twist.linear
        
        A = Point(h_x, h_y)
        Av = Point(h_v.x, h_v.y)
        obs = Point(r_x, r_y)
        obsv = Point(r_v.x, r_v.y)
        
        risk, collision, origin, left, right = compute_risk(A, obs, Av, obsv)
        
        msg = Risk()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
                
        msg.risk.data = risk
        msg.collision.data = collision
        msg.origin.x = origin.x
        msg.origin.y = origin.y
        msg.origin.z = 0
        msg.left.x = left.x
        msg.left.y = left.y
        msg.left.z = 0
        msg.right.x = right.x
        msg.right.y = right.y
        msg.right.z = 0
        self.pub_risk.publish(msg)
                
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    r = RiskClass()

    while not rospy.is_shutdown():
        rate.sleep()