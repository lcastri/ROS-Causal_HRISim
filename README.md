# ROS-Causal_HRISim
A Gazebo-based human-robot interaction simulator that accurately mimics HRI scenarios involving a [TIAGo](https://pal-robotics.com/robots/tiago/) robot and multiple pedestrians modelled using the [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros) ROS library. To better emulate human behaviours, we incorporated the option for user teleoperation (via keyboard) of a simulated person, not influenced by social forces.

<p float="left">
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/singleagent.gif" width="33%" height="33%" />
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/corridor.gif" width="33%" height="33%" />
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/multiagent.gif" width="33%" height="33%" />
</p>


The simulator has been designed to facilitate the setup of real-life HRI scenarios and the execution of causal analysis within them. For the latter, [ROS-Causal](https://github.com/lcastri/roscausal), a ROS-based causal analysis framework for HRI applications, has been integrated in the ROS-Causal_HRISim simulator.

## Features
* Individual and group walking pedestrian and other social activities simulation using pedsim
* Single-person teleoperation
* Customisable HRI scenario and world
* Customisable plans for the TIAGo robot
* Causal analysis

## How to use
### Build and run
After cloning the repository, use the following commands to build the Docker image and run it:
```
cd /path/to/ROS-Causal_HRISim
sudo ./build_run_docker.sh
```
Once the Docker image is built, you can use the following command to run the container:
```
cd /path/to/ROS-Causal_HRISim
sudo ./run_docker.sh
```
### Scenario setup and launch
Once inside the Docker container, run the following command to view the tmule file containing all the simulator parameters:
```
roscd hrisim_tmule/tmule
cat tiago_sim.yaml
```
Editable parameters:
* TIAGO_TYPE - represents the type of TIAGo;
* WORLD - world and map to load. It can be chosen among ["maze", "maze_corridor", "maze_corridor_withTurn", "maze_corridor_withDoors"]<br>
If you want to add your own WORLD, you can include your .world file in hrisim_gazebo/worlds and your map in hrisim_gazebo/tiago_maps.<br>
Note that the map must have the same name as the .world file;
* SCENARIO - pedsim scenario to load. It can be chosen among ["single_agent_avoidance", "multi_agent_avoidance"]<br>
If you want to add your own SCENARIO, you can include your .xml file in hrisim_pedsim/scenarios;
* MAX_HUMAN_SPEED - teleop humam max speed
* ROBOT_RADIUS - robot size
* HUMAN_x - teleop human init x-coordinate
* HUMAN_y - teleop human init y-coordinate
* SPAWN_AGENT - bit to decide whether to spawn agents driven by social forces (if present in the SCENARIO) 
* SPAWN_TELEOP_AGENT - bit to decide whether to spawn teleop agent
* SPAWN_TIMEOUT - spawn timeout

If you want to modify any of these parameters, you can edit the tiago_sim.yaml by:
```
nano tiago_sim.yaml
```
Once the tmule file is configured, you can start the simulator with the following command:
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

### Planning
The ROS-Causal_HRISim includes the [PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans) to define predefined plans for the TIAGo robot. The plan is a combination of actions and conditions that can be defined to create your own plan. Three different folders have been pre-created for plans, actions, and conditions, and they are:
* hrisim_plans
* hrisim_actions
* hrisim_conditions

For more details on how to define plans, actions, and conditions, visit the [PetriNetPlans](https://github.com/francescodelduchetto/PetriNetPlans) GitHub reposity.

### Causal analysis
The causal analysis is perform through the [ROS-Causal](https://github.com/lcastri/roscausal) ROS library. An example of causal model reconstructed from a HRI scenario involving the TIAGo robot and a single teleoperated human is shown in the following:
<p float="left">
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/singleagent.gif" width="56%" height="56%" />
    <img src="https://github.com/lcastri/ROS-Causal_HRISim/blob/main/images/causal_model_example.png" width="42%" height="42%" />
</p>

The raw data collected through the simulator and the corresponding post-processed data used in the causal analysis can be found [here](https://github.com/lcastri/ROS-Causal_HRISim/blob/main/data/experiment_20240131_234259.zip).

## Citation

If you found this useful for your work, please cite this papers:
```
@inproceedings{castri2024exp,
  title={Experimental Evaluation of ROS-Causal in Real-World Human-Robot Spatial Interaction Scenarios},
  author={Castri, Luca and Beraldo, Gloria and Mghames, Sariah and Hanheide, Marc and Bellotto, Nicola},
  booktitle={33nd IEEE International Conference on Robot and Human Interactive Communication (RO-MAN)},
  pages={},
  year={2024},
  organization={IEEE}
}
```


## Recent changes
| Version | Changes |
| :---: | ----------- |
| 1.0.0 | package released|
