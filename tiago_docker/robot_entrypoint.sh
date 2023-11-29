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

  echo "10.8.0.102 tiago-29c" >> /etc/hosts
  echo "10.8.0.106 tiago-89c" >> /etc/hosts
  echo "10.8.0.105 tiago-125c" >> /etc/hosts
  echo "192.168.1.29 tiago-29c" >> /etc/hosts
  echo "192.168.1.89 tiago-89c" >> /etc/hosts
  echo "192.168.1.125 tiago-125c" >> /etc/hosts

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
  echo "source ~/ros_ws/src/myTIAGo/scripts/connect_tiago.sh ${ROBOT_NUM} ${ETH}" >> ~/.bashrc
  catkin build
  source /home/lcastor/ros_ws/devel/setup.bash
  echo "function s(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago.yaml -W 3 launch ; }" >> ~/.bashrc
  echo "function t(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago.yaml terminate ; }" >> ~/.bashrc
  echo "function r(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago.yaml -W 3 relaunch ; }" >> ~/.bashrc

  exec su lcastor 
   

} || {

  echo "Container failed."
  exec "$@"

}