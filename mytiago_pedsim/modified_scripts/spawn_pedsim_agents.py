#!/usr/bin/env python
"""
Created on Mon Dec  2 17:03:34 2019

@author: mahmoud
"""

import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import *
from rospkg import RosPack
from pedsim_msgs.msg  import AgentStates

# xml file containing a gazebo model to represent agent, currently is represented by a cubic but can be changed
global xml_file

def actor_poses_callback(actors):
    global AGENT_SPAWN
    if not AGENT_SPAWN:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            spawn_model(actor_id, xml_string, "", model_pose, "world")
        rospy.logwarn("All autonomous agents have been spawn")
        AGENT_SPAWN = True
        
def teleop_actor_poses_callback(actors):
    global TELEOP_AGENT_SPAWN
    if not TELEOP_AGENT_SPAWN:
        for actor in actors.agent_states:
            actor_id = str( actor.id )
            actor_pose = actor.pose
            rospy.loginfo("Spawning model: actor_id = %s", actor_id)

            model_pose = Pose(Point(x= actor_pose.position.x,
                                y= actor_pose.position.y,
                                z= actor_pose.position.z),
                            Quaternion(actor_pose.orientation.x,
                                        actor_pose.orientation.y,
                                        actor_pose.orientation.z,
                                        actor_pose.orientation.w) )

            spawn_model(actor_id, xml_string, "", model_pose, "world")
        rospy.logwarn("All teleop agents have been spawn")
        TELEOP_AGENT_SPAWN = True

if __name__ == '__main__':

    rospy.init_node("spawn_pedsim_agents")
    rate = rospy.Rate(10)
    
    global AGENT_SPAWN, TELEOP_AGENT_SPAWN
    AGENT_SPAWN = True
    TELEOP_AGENT_SPAWN = False

    rospack1 = RosPack()
    pkg_path = rospack1.get_path('pedsim_gazebo_plugin')
    default_actor_model_file = pkg_path + "/models/actor_model.sdf"

    actor_model_file = rospy.get_param('~actor_model_file', default_actor_model_file)
    file_xml = open(actor_model_file)
    xml_string = file_xml.read()

    print("Waiting for gazebo services...")
    rospy.wait_for_service("gazebo/spawn_sdf_model")
    spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)
    print("service: spawn_sdf_model is available ....")
    rospy.Subscriber("/pedsim_simulator/simulated_agents", AgentStates, actor_poses_callback)
    rospy.Subscriber("/ped/control/gz_persons", AgentStates, teleop_actor_poses_callback)

    while not rospy.is_shutdown():
        if AGENT_SPAWN and TELEOP_AGENT_SPAWN:
            rospy.signal_shutdown("All agents have been spawned!")
        rate.sleep()
