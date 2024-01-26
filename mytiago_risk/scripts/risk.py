#!/usr/bin/env python

import rospy
from shapely.geometry import *
import math
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from pedsim_msgs.msg import TrackedPersons
from mytiago_risk.msg import Risk
import tf
from std_msgs.msg import Header


NODE_NAME = "mytiago_risk"
NODE_RATE = 10 # [Hz]
DIST_THRES = float(rospy.get_param("/hri/safe_distance", default = 5.0))


def get_2DPose(p: PoseWithCovarianceStamped):
    """
    Extracts x, y and theta from pose
    
    Args:
        p (PoseWithCovarianceStamped): pose
    
    Returns:
        tuple: x, y, theta
    """
    x = p.pose.pose.position.x
    y = p.pose.pose.position.y
    
    q = (
        p.pose.pose.orientation.x,
        p.pose.pose.orientation.y,
        p.pose.pose.orientation.z,
        p.pose.pose.orientation.w
    )
    
    m = tf.transformations.quaternion_matrix(q)
    _, _, yaw = tf.transformations.euler_from_matrix(m)
    return x, y, yaw


class OnlineRisk():
    
    def __init__(self) -> None:
        """
        Risk constructor
        """

        # Risk publisher     
        self.pub_risk = rospy.Publisher('/hri/risk', Risk, queue_size=10)
        
        # Odometry subscriber
        sub_odom = message_filters.Subscriber("/mobile_base_controller/odom", Odometry)
        
        # Person subscriber
        sub_people = message_filters.Subscriber('/ped/control/teleop_persons', TrackedPersons)
                
        # Robot pose subscriber
        sub_robot_pose = message_filters.Subscriber('/robot_pose', PoseWithCovarianceStamped)
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_odom, 
                                                                sub_people, 
                                                                sub_robot_pose], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)

        self.ats.registerCallback(self.get_data)
         
        
    def compute_risk(self, A: Point, obs: Point, Av: Point, obsv: Point):
        risk = 0
        collision = False
        
        # Calculate relative velocity vector
        Vrel = Point(obsv.x - Av.x, obsv.y - Av.y)
               
        # Compute the slope of the line AB and line PAB ⊥ AB
        slope_AB = (obs.y - A.y) / (obs.x - A.x)  # Rise over run
        slope_PAB = -1 / slope_AB

        # Choose distances to the left and right of point B
        inc_size = 1.5

        # Calculate coordinates for two points along the perpendicular line
        # Calculate the change in x and y based on the fixed horizontal distance
        delta_x = inc_size / (1 + slope_PAB ** 2) ** 0.5
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
        

    def get_data(self, odom: Odometry, 
                       people: TrackedPersons, 
                       robot_pose: PoseWithCovarianceStamped):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            people (TrackedPersons): tracked people
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        
        # Robot 2D pose (x, y, theta) and velocity
        r_x, r_y, r_theta = get_2DPose(robot_pose)
        r_v = abs(odom.twist.twist.linear.x)
        
        # Human 2D pose (x, y, theta) and velocity
        p = people.tracks[0]            
        h_x = p.pose.pose.position.x
        h_y = p.pose.pose.position.y
        h_theta = p.pose.pose.orientation.z
        h_v = abs(p.twist.twist.linear.x)
        
        A = Point(h_x, h_y)
        Av = Point(h_v*math.cos(h_theta), h_v*math.sin(h_theta))
        obs = Point(r_x, r_y)
        obsv = Point(r_v*math.cos(r_theta), r_v*math.sin(r_theta))
        
        risk, collision, origin, left, right = self.compute_risk(A, obs, Av, obsv)
        
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
    
    r = OnlineRisk()

    while not rospy.is_shutdown():
        rate.sleep()