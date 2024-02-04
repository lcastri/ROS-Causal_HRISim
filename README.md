# ROS-Causal_HRISim
A Gazebo-based human-robot interaction simulator that accurately mimics HRI scenarios involving a [TIAGo](https://pal-robotics.com/robots/tiago/) robot and multiple pedestrians modelled using the [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) ROS library. To better emulate human behaviours, we incorporated the option for user teleoperation (via keyboard) of a simulated person, not influenced by social forces.

<p float="left">
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/singleagent.gif" width="33%" height="33%" />
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/corridor.gif" width="33%" height="33%" />
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/multiagent.gif" width="33%" height="33%" />
</p>


The simulator has been designed to facilitate the setup of real-life HRI scenarios and the execution of causal analysis within them. For the latter, [ROS-Causal](https://github.com/lcastri/roscausal), a ROS-based causal analysis framework for HRI applications, has been integrated in the ROS-Causal_HRISim simulator.

## Features
* Individual and group walking pedestrian using social force model
* Social activities simulation
* Single person teleoperation
* Personalisable HRI scenario and world
* Causal analysis

## How to use
### Build and run
After cloning the repository, use the following commands to build the Docker image and run it:
```
cd /path/to/ROS-Causal_HRISim
sudo ./build_run_docker.sh
```
Once built the Docker image, you can use the following command to only run the docker:
```
cd /path/to/ROS-Causal_HRISim
sudo ./run_docker.sh
```
### Scenario setup and launch
Once inside the Docker, run the following command to choose the map:
```
export WORLD="MAP_NAME"
```
the available maps are:
- "maze";
- "maze_corridor";
- "maze_corridor_withTurn";
- "maze_corridor_withDoors".

Run the following command to start a tmule session:
```
tm-start
```
to visualise the tmule session
```
tm-show
```
once inside the tmule, run the following command to change panel:
```
Ctrl+b
panel number [0-6]
```
and finally to stop it
```
Ctrl+b
panel number 0
tm-stop
```

## Recent changes
| Version | Changes |
| :---: | ----------- |
| 1.0.0 | package released|