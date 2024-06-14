#!/usr/bin/env python

import rospy
from shapely.geometry import *
import math
import message_filters
from hrisim_risk.msg import Risk
from roscausal_msgs.msg import RobotState, HumanState, Humans
from std_msgs.msg import Header


NODE_NAME = "hrisim_risk"
NODE_RATE = 10 # [Hz]

def compute_risk(A: Point, obs: Point, Av: Point, obsv: Point):
    risk = math.sqrt(Av.x**2 + Av.y**2)
    collision = False
    
    # Calculate relative velocity vector
    Vrel = Point(obsv.x - Av.x, obsv.y - Av.y)
           
    # Compute the slope of the line AB and line PAB ⊥ AB
    slope_AB = (obs.y - A.y) / (obs.x - A.x)  # Rise over run
    slope_PAB = -1 / slope_AB
    # Choose distances to the left and right of point B

    # Calculate coordinates for two points along the perpendicular line
    # Calculate the change in x and y based on the fixed horizontal distance
    delta_x = OBS_SIZE / (1 + slope_PAB ** 2) ** 0.5
    delta_y = delta_x * slope_PAB
    # Calculate coordinates for two points along the perpendicular line
    left = Point(obs.x - delta_x, obs.y - delta_y)
    right = Point(obs.x + delta_x, obs.y + delta_y)
    # Cone
    cone_origin = Point(A.x, A.y)               
    cone = Polygon([cone_origin, left, right])
    
    P = Point(cone_origin.x + Av.x, cone_origin.y + Av.y)
    collision = P.within(cone) and A.distance(obs) < SAFE_DIST
    if collision:
        time_collision_measure = A.distance(obs) / math.sqrt(Vrel.x**2 + Vrel.y**2)
        steering_effort_measure = min(P.distance(LineString([cone_origin, left])), P.distance(LineString([cone_origin, right])))           
        risk = risk + 1/time_collision_measure + steering_effort_measure
                
    return math.exp(risk), collision, cone_origin, left, right


class RiskClass():
    
    def __init__(self) -> None:
        """
        RiskClass constructor
        """
        self.firstcb = True
        self.A = None
        self.Av = None
        self.obs = None
        self.obsv = None
        
        # Risk publisher     
        self.pub_risk = rospy.Publisher('/hri/risk', Risk, queue_size=10)
        
        # Robot subscriber
        sub_robot = message_filters.Subscriber("/roscausal/robot", RobotState)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/roscausal/humans', Humans)
                
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_robot, 
                                                                sub_people], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)


        self.ats.registerCallback(self.cb_risk)
        
    
    def extract_data(self, robot, people):
        obstacles = list()
        
        # Teleop human 2D pose (x, y, theta) and velocity
        for person in people.humans:
            if person.id == SELAGENT_ID:
                h_x = person.pose2D.x
                h_y = person.pose2D.y
                h_v = person.twist.linear
                
                self.A = Point(h_x, h_y)
                self.Av = Point(h_v.x, h_v.y)
                    
                # Robot 2D pose (x, y, theta) and velocity
                r_x = robot.pose2D.x
                r_y = robot.pose2D.y
                r_v = robot.twist.linear
                obstacles.append((r_x,r_y,r_v))
                for person in people.humans:
                    if person.id != SELAGENT_ID:
                        x = person.pose2D.x
                        y = person.pose2D.y
                        v = person.twist.linear
                        obstacles.append((x,y,v))
                        
                # Select the closest obstacle to the selected human
                closest_obs = None
                min_distance = float('inf')
                for obs_x, obs_y, obs_v in obstacles:
                    distance = ((obs_x - h_x) ** 2 + (obs_y - h_y) ** 2) ** 0.5
                    if distance < min_distance:
                        min_distance = distance
                        closest_obs = (obs_x, obs_y, obs_v)
                    
                self.obs = Point(closest_obs[0], closest_obs[1])
                self.obsv = Point(closest_obs[2].x, closest_obs[2].y)
                return
            
        self.obs = None
        self.obsv = None
                       
    # NOTE: uncomment this callback if you want the risk at time t compute with data at time t-1 
    # def cb_risk(self, robot: RobotState, people: Humans):
    #     """
    #     Synchronized callback

    #     Args:
    #         robot (RobotState): robot state
    #         people (Humans): people state
    #     """
    #     if self.firstcb:
    #         self.extract_data(robot, people)
    #         self.firstcb = False
        
    #     else:
    #         if self.obs is not None and self.obsv is not None:
    #             risk, collision, origin, left, right = compute_risk(self.A, self.obs, self.Av, self.obsv)
                
    #             msg = Risk()
    #             msg.header = Header()
    #             msg.header.stamp = rospy.Time.now()
                        
    #             msg.risk.data = risk
    #             msg.collision.data = collision
    #             msg.origin.x = origin.x
    #             msg.origin.y = origin.y
    #             msg.origin.z = 0
    #             msg.left.x = left.x
    #             msg.left.y = left.y
    #             msg.left.z = 0
    #             msg.right.x = right.x
    #             msg.right.y = right.y
    #             msg.right.z = 0
    #             self.pub_risk.publish(msg)
                
    #         self.extract_data(robot, people)
            
            
    # NOTE: uncomment this callback if you want the risk at time t compute with data at time t
    def cb_risk(self, robot: RobotState, people: Humans):
        """
        Synchronized callback

        Args:
            robot (RobotState): robot state
            people (Humans): people state
        """
        self.extract_data(robot, people)

        if self.obs is not None and self.obsv is not None:
            risk, collision, origin, left, right = compute_risk(self.A, self.obs, self.Av, self.obsv)
                    
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

    SAFE_DIST = float(rospy.get_param("/hri/safe_distance", default = 5.0))
    OBS_SIZE = float(rospy.get_param("/hri/obs_size", default = 2.5))
    SELAGENT_ID = int(rospy.get_param("/hri/selected_agent_id"))
        
    r = RiskClass()

    while not rospy.is_shutdown():
        rate.sleep()