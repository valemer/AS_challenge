# AS_challenge

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
rosrun basic_waypoint_pkg manual_control.py
```

