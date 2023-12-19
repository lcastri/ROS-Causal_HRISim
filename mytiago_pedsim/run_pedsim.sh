  export SCENARIO="single_agent_avoidance"
  export ROBOT_MODE=1
  export  MAX_ROBOT_SPEED=1.5
  export  ROBOT_RADIUS=2
  export  WITH_PED="true"
  export  KBD_TELEOP="true"
  export  HUMAN_x=5
  export  HUMAN_y=5
  
 roslaunch mytiago_pedsim mysimulator.launch scenario:=$SCENARIO robot_mode:=$ROBOT_MODE max_robot_speed:=$MAX_ROBOT_SPEED robot_radius:=$ROBOT_RADIUS kbd_teleop:=$KBD_TELEOP with_ped:=$WITH_PED ped_x:=$HUMAN_x ped_y:=$HUMAN_y ped_theta:=0
