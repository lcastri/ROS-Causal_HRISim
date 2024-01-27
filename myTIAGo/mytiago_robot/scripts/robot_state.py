#!/usr/bin/env python

import rospy
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from mytiago_robot.msg import RobotState
import tf
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_vector3
from std_msgs.msg import Header
from geometry_msgs.msg import Vector3Stamped, Pose2D, Twist


NODE_NAME = "mytiago_robot"
NODE_RATE = 10 # [Hz]


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
    return Pose2D(x, y, yaw)


class RobotStateClass():
    
    def __init__(self) -> None:
        """
        RobotState constructor
        """
        # Risk publisher     
        self.pub_robot_state = rospy.Publisher('/hri/robot_state', RobotState, queue_size=10)
        
        # Odometry subscriber
        sub_odom = message_filters.Subscriber("/mobile_base_controller/odom", Odometry)
                        
        # Robot pose subscriber
        sub_robot_pose = message_filters.Subscriber('/robot_pose', PoseWithCovarianceStamped)
                
        # Init synchronizer and assigning a callback 
        self.ats = message_filters.ApproximateTimeSynchronizer([sub_odom,  
                                                                sub_robot_pose], 
                                                                queue_size = 10, slop = 0.1,
                                                                allow_headerless = True)
        
        # Create a TransformListener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)

        self.ats.registerCallback(self.get_data)
                       

    def get_data(self, odom: Odometry,
                       robot_pose: PoseWithCovarianceStamped):
        """
        Synchronized callback

        Args:
            robot_odom (Odometry): robot odometry
            robot_pose (PoseWithCovarianceStamped): robot pose
        """
        transform = self.tf_buffer.lookup_transform('map', 'base_footprint', rospy.Time(), rospy.Duration(0.5))
        original_vector = Vector3Stamped()
        original_vector.header.frame_id = "base_footprint"
        original_vector.vector.x = odom.twist.twist.linear.x  # Your original Vector3 values
        original_vector.vector.y = odom.twist.twist.linear.y
        original_vector.vector.z = odom.twist.twist.linear.z
        r_v = do_transform_vector3(original_vector, transform)
        
        original_vector = Vector3Stamped()
        original_vector.header.frame_id = "base_footprint"
        original_vector.vector.x = odom.twist.twist.angular.x  # Your original Vector3 values
        original_vector.vector.y = odom.twist.twist.angular.y
        original_vector.vector.z = odom.twist.twist.angular.z
        r_w = do_transform_vector3(original_vector, transform)
        
        twist = Twist()
        twist.linear.x = r_v.vector.x
        twist.linear.y = r_v.vector.y
        twist.linear.z = r_v.vector.z
        twist.angular.x = r_w.vector.x
        twist.angular.y = r_w.vector.y
        twist.angular.z = r_w.vector.z
        
               
        msg = RobotState()
        msg.header = Header()
        msg.header.stamp = rospy.Time.now()
                
        msg.frame_id = 'map'
        msg.pose2D = get_2DPose(robot_pose)
        msg.twist = twist
        self.pub_robot_state.publish(msg)
                
        

if __name__ == '__main__':
    
    # Node
    rospy.init_node(NODE_NAME, anonymous=True)
    rate = rospy.Rate(NODE_RATE)
    
    r = RobotStateClass()

    while not rospy.is_shutdown():
        rate.sleep()