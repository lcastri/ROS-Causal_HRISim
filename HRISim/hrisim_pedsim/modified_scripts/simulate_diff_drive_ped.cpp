#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <pedsim_msgs/AgentStates.h>
#include <pedsim_msgs/AgentState.h>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

double g_updateRate, g_simulationFactor;
int selectedAgentId;
std::string g_worldFrame, g_robotFrame;
geometry_msgs::Twist g_currentTwist;
tf::Transform g_currentPose;
boost::shared_ptr<tf::TransformBroadcaster> g_transformBroadcaster;
boost::mutex mutex;
ros::Publisher trackedPersonsPublisher; // Declare as a global variable
ros::Publisher GZPersonsPublisher; // Declare as a global variable


void publishTeleopPersons(double dt, double x, double y, double theta, double v, double omega) {
  // Create TrackedPersons message and TrackedPerson message
  pedsim_msgs::TrackedPersons trackedPersonsMsg;
  pedsim_msgs::TrackedPerson trackedPersonMsg;

  // Fill TrackedPerson message with calculated pose and other information
  trackedPersonMsg.track_id = selectedAgentId;
  trackedPersonMsg.is_occluded = false;
  trackedPersonMsg.is_matched = true;
  trackedPersonMsg.detection_id = selectedAgentId;
  trackedPersonMsg.age = ros::Duration(dt);

  trackedPersonMsg.pose.pose.position.x = x;
  trackedPersonMsg.pose.pose.position.y = y;
  trackedPersonMsg.pose.pose.position.z = 0.0;
  trackedPersonMsg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  trackedPersonMsg.twist.twist.linear.x = v*cos(theta);
  trackedPersonMsg.twist.twist.linear.y = v*sin(theta);
  trackedPersonMsg.twist.twist.angular.z = omega;

  // Add the TrackedPerson to TrackedPersons
  trackedPersonsMsg.header.stamp = ros::Time::now();
  trackedPersonsMsg.header.frame_id = g_worldFrame;
  trackedPersonsMsg.tracks.push_back(trackedPersonMsg);

  // Publish the TrackedPersons message
  trackedPersonsPublisher.publish(trackedPersonsMsg);
}

void publishGZPersons(double x, double y, double theta, double v, double omega) {
  // Create AgentStates message and AgentState message
  pedsim_msgs::AgentStates agentStatesMsg;
  pedsim_msgs::AgentState agentStateMsg;

  // Fill agentStateMsg message with calculated pose and other information
  agentStateMsg.id = selectedAgentId;
  agentStateMsg.type = 0;
  agentStateMsg.social_state = "individual_moving";

  agentStateMsg.pose.position.x = x;
  agentStateMsg.pose.position.y = y;
  agentStateMsg.pose.position.z = 0.0;
  agentStateMsg.pose.orientation = tf::createQuaternionMsgFromYaw(theta);

  agentStateMsg.twist.linear.x = v*cos(theta);
  agentStateMsg.twist.linear.y = v*sin(theta);
  agentStateMsg.twist.angular.z = omega;

  // Add the TrackedPerson to TrackedPersons
  agentStatesMsg.header.stamp = ros::Time::now();
  agentStatesMsg.header.frame_id = g_worldFrame;
  agentStatesMsg.agent_states.push_back(agentStateMsg);

  // Publish the TrackedPersons message
  GZPersonsPublisher.publish(agentStatesMsg);
}


/// Simulates robot motion of a differential-drive robot with translational and
/// rotational velocities as input
/// These are provided in the form of a geometry_msgs::Twist, e.g. by
/// turtlebot_teleop/turtlebot_teleop_key.
/// The resulting robot position is published as a TF transform from world -->
/// base_footprint frame.
void updateLoop() {
  ros::Rate rate(g_updateRate);
  const double dt = g_simulationFactor / g_updateRate;
  while (true) {
    // Get current pose
    double x = g_currentPose.getOrigin().x();
    double y = g_currentPose.getOrigin().y();
    double theta = tf::getYaw(g_currentPose.getRotation());

    // Get requested translational and rotational velocity
    double v, omega;
    {
      boost::mutex::scoped_lock lock(mutex);
      v = g_currentTwist.linear.x;
      omega = g_currentTwist.angular.z;
    }

    // Simulate robot movement
    x += cos(theta) * v * dt;
    y += sin(theta) * v * dt;
    theta += omega * dt;

    // Update pose
    g_currentPose.getOrigin().setX(x);
    g_currentPose.getOrigin().setY(y);
    g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, theta));

    // Broadcast transform
    g_transformBroadcaster->sendTransform(tf::StampedTransform(
        g_currentPose, ros::Time::now(), g_worldFrame, g_robotFrame));

    // publish /teleop_persons and /gz_persons
    publishTeleopPersons(dt, x, y, theta, v, omega);
    publishGZPersons(x, y, theta, v, omega);

    rate.sleep();
  }
}

void onTwistReceived(const geometry_msgs::Twist::ConstPtr& twist) {
  boost::mutex::scoped_lock lock(mutex);
  g_currentTwist = *twist;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "simulate_diff_drive_ped");
  ros::NodeHandle nodeHandle("");
  ros::NodeHandle privateHandle("~");

  // Process parameters
  privateHandle.param<std::string>("world_frame", g_worldFrame, "odom");
  privateHandle.param<std::string>("ped_frame", g_robotFrame,
                                   "ped_base_footprint");

  privateHandle.param<double>("/pedsim_simulator/simulation_factor", g_simulationFactor,
                              1.0);  // set to e.g. 2.0 for 2x speed
  privateHandle.param<double>("/pedsim_simulator/update_rate", g_updateRate, 25.0);  // in Hz

  double initialX = 0.0, initialY = 0.0, initialTheta = 0.0;
  privateHandle.param<double>("ped_x", initialX, 0.0);
  privateHandle.param<double>("ped_y", initialY, 0.0);
  privateHandle.param<double>("ped_theta", initialTheta, 0.0);

  g_currentPose.getOrigin().setX(initialX);
  g_currentPose.getOrigin().setY(initialY);
  g_currentPose.setRotation(tf::createQuaternionFromRPY(0, 0, initialTheta));

  // Read selected_agent_id parameter
  if (!privateHandle.getParam("/hri/selected_agent_id", selectedAgentId)) {
    ROS_ERROR("Failed to read parameter '/hri/selected_agent_id'");
    return 1;
  }
  
  // Create ROS subscriber and TF broadcaster
  g_transformBroadcaster.reset(new tf::TransformBroadcaster());
  ros::Subscriber twistSubscriber =
      nodeHandle.subscribe<geometry_msgs::Twist>("cmd_vel", 3, onTwistReceived);

  // Create a publisher for the TrackedPersons message
  trackedPersonsPublisher = nodeHandle.advertise<pedsim_msgs::TrackedPersons>("teleop_persons", 10);
  GZPersonsPublisher = nodeHandle.advertise<pedsim_msgs::AgentStates>("gz_persons", 10);

  // Run
  boost::thread updateThread(updateLoop);
  ros::spin();
}
