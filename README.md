# AS_challenge


# AS_challenge

```
AS_challenge/
├── catkin_ws/
│   ├── src/
│   │   ├── planning/
│   │   ├── perception/
│   │   ├── simulation/
│   │   └── config_rviz/          
```

## System Components

`planning`
> Cave exploration algorithm, BFS planning (used for flying back after finding all lanterns), trajectory generation

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

