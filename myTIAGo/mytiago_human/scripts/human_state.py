#!/usr/bin/env python

import math
import numpy as np
import rospy
from mytiago_human.msg import HumanState, Risk
from mytiago_robot.msg import RobotState
from pedsim_msgs.msg import TrackedPersons
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D, Twist, PoseWithCovariance
from shapely.geometry import *


NODE_NAME = "mytiago_human"
NODE_RATE = 10 # [Hz]
DIST_THRES = float(rospy.get_param("/hri/safe_distance", default = 5.0))
INC_SIZE = float(rospy.get_param("/hri/obs_size", default = 2.5))
NXY = float(rospy.get_param("/human_state/nxy", default = 0.05))
NTHETA = float(rospy.get_param("/human_state/ntheta", default = 0.1))
NV = float(rospy.get_param("/human_state/nv", default = 0.075))
NW = float(rospy.get_param("/human_state/nw", default = 0.08))
ADDNOISE = bool(rospy.get_param("/human_state/addnoise", default = False))

def wrap(angle, lower_bound, upper_bound):
    """
    Wrap an angle to be within the specified bounds.

    Args:
        angle (float): The angle to be wrapped.
        lower_bound (float): The lower bound for the angle.
        upper_bound (float): The upper bound for the angle.

    Returns:
        float: The wrapped angle.
    """
    range_width = upper_bound - lower_bound
    wrapped_angle = (angle - lower_bound) % range_width + lower_bound

    return wrapped_angle


def get_2DPose(p: PoseWithCovariance):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (PoseWithCovarianceStamped): pose
    
    Returns:
        tuple: x, y, theta
    """
    x = p.pose.position.x
    y = p.pose.position.y
    
    q = (
        p.pose.orientation.x,
        p.pose.orientation.y,
        p.pose.orientation.z,
        p.pose.orientation.w
    )
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return Pose2D(x, y, yaw)


def compute_risk(A: Point, obs: Point, Av: Point, obsv: Point, noise):
    risk = noise
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


class HumanStateClass():
    
    def __init__(self) -> None:
        """
        HumanState constructor
        """
        self.x = None
        self.y = None
        self.theta = None
        self.v = None
        self.w = None
        
        self.robot = None
        
        # Risk publisher     
        self.pub_human_state = rospy.Publisher('/hri/human_state', HumanState, queue_size=10)
        
        # TrackedPersons subscriber
        rospy.Subscriber("/ped/control/teleop_persons", TrackedPersons, self.get_data)
        
        # RobotState subscriber
        rospy.Subscriber("/hri/robot_state", RobotState, self.cb_robot)
        
        
    def cb_robot(self, robot):
        """
        Robot state callback

        Args:
            robot (RobotState): robot state
        """
        self.robot = robot
                                                              

    def get_data(self, person: TrackedPersons):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        person = person.tracks[0]
        state = get_2DPose(person.pose)
        self.x = state.x
        self.y = state.y
        self.theta = state.theta
        self.v = person.twist.twist.linear
        self.w = person.twist.twist.angular
        
        if ADDNOISE:
            self.x += np.random.normal(0, NXY)
            self.y += np.random.normal(0, NXY)
            self.theta += np.random.normal(0, NTHETA)
            self.v.x += np.random.normal(0, NXY) 
            self.v.y += np.random.normal(0, NXY) 
            self.v.z += np.random.normal(0, NXY) 
            self.w.x += np.random.normal(0, NW) 
            self.w.y += np.random.normal(0, NW) 
            self.w.z += np.random.normal(0, NW) 
        
        goal = rospy.get_param('/hri/human_goal', default=[state.x, state.y])
        
        self.dg = math.dist([self.x, self.y], goal)
        self.thetag = self.heading(goal)      
    
        # msg
        msg = HumanState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
        msg.frame_id = 'map'
        msg.pose2D = Pose2D(self.x, self.y, self.theta)
        
        twist = Twist()
        twist.linear.x = self.v.x
        twist.linear.y = self.v.y
        twist.linear.z = self.v.z
        twist.angular.x = self.w.x
        twist.angular.y = self.w.y
        twist.angular.z = self.w.z        
        msg.twist = twist
        msg.dg.data = self.dg
        msg.thetag.data = self.thetag
        
        if self.robot is not None:
            nrisk = np.random.normal(0, NXY) if ADDNOISE else 0
            r, collision, origin, left, right = self.risk(self.robot, nrisk)
            risk = Risk()
            risk.header = Header()
            risk.header.stamp = rospy.Time.now()        
            risk.risk.data = r
            risk.collision.data = collision
            risk.origin.x = origin.x
            risk.origin.y = origin.y
            risk.origin.z = 0
            risk.left.x = left.x
            risk.left.y = left.y
            risk.left.z = 0
            risk.right.x = right.x
            risk.right.y = right.y
            risk.right.z = 0
            msg.risk = risk
        
        self.pub_human_state.publish(msg)
        
        
    def heading(self, g):
        """
        heading angle
        
        Args:
            g (Point): target point
            
        Returns:
            float: heading angle
        """
        
        angle = wrap(2*np.pi - wrap(math.atan2(g[1]-self.y, g[0]-self.x) - wrap(self.theta, 0, 2*np.pi), 0, 2*np.pi), -np.pi, np.pi)
        return angle
    
    
    def risk(self, robot: RobotState, noise):
        """
        Risk

        Args:
            robot (RobotState): robot state
            people (TrackedPersons): tracked people
        """
        # Robot 2D pose (x, y, theta) and velocity
        r_x = robot.pose2D.x
        r_y = robot.pose2D.y
        r_v = robot.twist.linear

        # Human 2D pose (x, y, theta) and velocity       
        A = Point(self.x, self.y)
        Av = Point(self.v.x, self.v.y)
        obs = Point(r_x, r_y)
        obsv = Point(r_v.x, r_v.y)
        
        return compute_risk(A, obs, Av, obsv, noise)
        
                
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    r = HumanStateClass()

    while not rospy.is_shutdown():
        rate.sleep()