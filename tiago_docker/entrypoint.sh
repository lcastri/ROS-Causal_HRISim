#!/bin/bash

set -e

# setup environment
source "$HOME/.bashrc"

echo " "
echo "###"
echo "### This is the TIAGo container!"
echo "###"
echo " "

{

  echo "Container is now running."
  echo " "
   echo "function tm-start(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago_sim.yaml -W 3 launch ; }" >> ~/.bashrc
   echo "function tm-stop(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago_sim.yaml terminate ; }" >> ~/.bashrc
   echo "function tm-restart(){  tmule -c ~/ros_ws/src/myTIAGo/mytiago_tmule/tmule/tiago_sim.yaml -W 3 relaunch ; }" >> ~/.bashrc
   echo "function tm-show(){  tmux a -t tiago_sim ; }" >> ~/.bashrc

   cd ~/ros_ws
   catkin build
   source ~/ros_ws/devel/setup.bash
   exec "/bin/bash"

} || {

  echo "Container failed."
  exec "$@"

}
