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
│   ├── build/                 
│   └── devel/                 
```

## System Components

`planning`
> Cave exploration algorithm, BFS planning, trajectory generation

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

## Start Simulation
```
cd catkin_ws
. devel/setup.bash
roslaunch simulation simulation.launch
```

## Start Planning
```
cd catkin_ws
. devel/setup.bash
roslaunch planning planning.launch
```

## Start RVIZ
```
cd catkin_ws
. devel/setup.bash
roslaunch config_rviz rviz.launch
```

## Start manual control
```
cd catkin_ws
. devel/setup.bash
rosrun planning manual_control.py
```

## Launch Files

### all_in_one_with_planning.launch
starts the simulation, planning, and RVIZ 
```
roslaunch simulation all_in_one_with_planning.launch
```

### all_in_one_with_manual_control.launch
starts the simulation, RVIZ, and manual control.
```
roslaunch simulation all_in_one_with_manual_control.launch
```

