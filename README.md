# ROS-Causal_HRISim
build and run the docker:
```
sudo ./build_run_docker.sh
```
only run the docker:
```
sudo ./run_docker.sh
```
once inside the docker, run the following command to choose the map:
```
export WORLD="maze"
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
to view the tmule session
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
