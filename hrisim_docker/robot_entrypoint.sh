#!/bin/bash

ROBOT_NUM=$1
ETH=$2

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This is the TIAGo container!"
echo "###"
echo " "

{
  echo -e "10.68.0.1\ttiago-125c" >> /etc/hosts

  echo "Container is now running."
  echo " "
  if [ -z "$ROBOT_NUM" ]; then
    echo "ERROR: No argument supplied!"
    echo " "
    echo "You should run as:   ./run_robot_docker.sh ROBOT_NUM ETH"
    echo "         (example:   ./run_robot_docker.sh 125 true)"
    echo " "
    exit 1
  else
    echo "It will connect to tiago ${ROBOT_NUM}"
  fi 
  echo "function tm-start(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago.yaml -W 3 launch ; }" >> ~/.bashrc
  echo "function tm-stop(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago.yaml terminate ; }" >> ~/.bashrc
  echo "function tm-restart(){  tmule -c ~/ros_ws/src/HRISim/hrisim_tmule/tmule/tiago.yaml -W 3 relaunch ; }" >> ~/.bashrc
  echo "function tm-show(){  tmux a -t tiago ; }" >> ~/.bashrc
  echo "function connect(){  source ~/ros_ws/src/HRISim/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH} ; }" >> ~/.bashrc
  
  echo "source ~/ros_ws/src/HRISim/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH}" >> ~/.bashrc
    
  source /opt/ros/noetic/setup.bash
  cd ~/ros_ws
  catkin build
  source ~/ros_ws/devel/setup.bash
  echo "source ~/ros_ws/devel/setup.bash" >> ~/.bashrc
  exec "/bin/bash"

} || {

  echo "Container failed."
  exec "$@"

}