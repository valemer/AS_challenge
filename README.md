# AS_challenge


This project requires ROS Noetic (installation in build script included) and is tested on Ubuntu 20.04. 
If you face problems during the simulation, please make sure to close other applications as the project makes use of parallel nodes.

The structure of the packages is:

```
AS_challenge/
├── catkin_ws/
│   ├── src/
│   │   ├── control/
│   │   ├── mapping/
│   │   ├── perception/
│   │   ├── planning/
│   │   ├── simulation/
│   │   └── urilities/          
```

## System Components

`planning`
> State Machine, Cave exploration algorithm, BFS planning (used for flying back after finding all lanterns), trajectory generation

`perception` 
> Pointcloud processing, junction and lantern detection

`simulation`
> Cave environment physics and rendering

`config_rviz`
> Visualization tools configuration


## Install required pkgs, build ws, and unzip sim
```
chmod +x build_and_extract_sim.sh
./build_and_extract_sim.sh
```

## Software Start

### Launch complete software stack
starts the simulation, drone software, and RVIZ
```
roslaunch simulation all_in_one_with_planning.launch
```

### Start Simulation
```
cd catkin_ws
. devel/setup.bash
roslaunch simulation simulation.launch
```

### Start Planning
```
cd catkin_ws
. devel/setup.bash
roslaunch planning planning.launch
```

### Start RVIZ
```
cd catkin_ws
. devel/setup.bash
roslaunch config_rviz rviz.launch
```

### Start manual control
```
cd catkin_ws
. devel/setup.bash
rosrun planning manual_control.py
```

### Start complete software stack without planning but manual control
starts the simulation, RVIZ, and manual control.
```
roslaunch simulation all_in_one_with_manual_control.launch
```

## Replay recorded Rosbag
If you have problems (e.g. limited computing power) and still want to see the result,
you can play the rosbag. We used a strong compression which causes some message losses but the drones behavior is still observable.
When you replay it, first start RVIZ as described above and in another terminal start the rosbag with speed=5x:
```
cd catkin_ws
. devel/setup.bash
rosbag play -r 5 ../rosbag.bag 
```