# mytiago_robot/msg/RobotState.msg

Header header
string frame_id
geometry_msgs/Pose2D pose2D
geometry_msgs/Twist twist
std_msgs/Float32 dg
std_msgs/Float32 thetag
mytiago_human/Risk risk