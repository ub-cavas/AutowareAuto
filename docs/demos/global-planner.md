Starting and testing the global planner {#avp-demo-test-global-planner}
=======================================

## How to start the stack

The following instructions were tested for git hash `8431b1ba47a8e` on `origin/integration`.

Start simulation as described [in the docs](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html), then configure it properly

1. Maps: use this [map link](https://assets.dev.lgsvlsimulator.com/d5b8bb0b7f49875a8a4bbf83c50b3a4fe53779c7/environment_AutonomouStuff)
1. Vehicles: Select `ROS2 native` bridge type and paste content of `AutowareAuto/lgsvl-sensors.json` into the `Sensors` text box
1. Simulations: In `General` tab, `Select Cluster = Local Machine` and untick any boxes. In `Map & Vehicles` tab, ensure to untick `Run simulation in interactive mode`. In `Traffic` tab, untick all selection. `Weather` is irrelevant

*terminal 1*
```
> ade enter
ade$ cd AutowareAuto && source install/setup.bash # assumes all packages are already in install
ade$ /opt/lgsvl/simulator
# start sim according to instructions but don't drive away yet to make sure we can localize ourselves
```

*terminal 2*
```
> ade enter
ade$ cd AutowareAuto && source install/setup.bash
ade$ colcon build --packages-up-to autoware_auto_avp_demo
ade$ ros2 launch autoware_auto_avp_demo ms3_sim.launch.py
```

*terminal 3*
```
> ade enter
ade$ cd AutowareAuto && source install/setup.bash
ade$ colcon build --packages-up-to lanelet2_global_planner_node
ade$ ros2 launch lanelet2_global_planner_node lanelet2_global_planner.launch.py
```

*terminal 3*
```
> ade enter
ade$ ros2 topic echo /planning/global_path
```

*terminal 4*
```
> ade enter
ade$ 
ros2 topic pub --once /planning/goal_pose geometry_msgs/msg/PoseStamped "{
header:
  {stamp:
    {sec: 1600775035,
    nanosec: 496432027},
  frame_id: map},
pose:
  {position:
    {x: -77.87589263916016,
    y: -18.580652236938477,
    z: 0.0},
  orientation:
    {x: 0.0,
    y: 0.0,
    z: 0.14149930538744868,
    w: 0.9899383549367453}}}
"
```

The path with lane IDs should be output in *terminal 3**. 

**Note** To choose a different parking spot, click `2D Nav Goal` in `rviz` and listen to the message with 

    ros2 topic echo /goal_pose

## Passing metrics

The output message looks like 

```
...
primitives:
- id: 8252
  primitive_type: parking
- id: 9074
  primitive_type: drivable_area
- id: 6581
  primitive_type: lane
  
# lots of lanes omitted  

- id: 6700
  primitive_type: lane
- id: 7957
  primitive_type: drivable_area
- id: 9824
  primitive_type: parking
```

To check if the route is reasonable, open the OSM map `autonomousstuff_parking_lot.osm` in a text editor and search for `way id='9824`. It references a node, the center of the entrance line, `9831` in this case. Searching for its coordinates, they are 

      <node id='9831' visible='true' version='1' lat='37.38065126054' lon='-121.90931385745'>

To graphically inspect this, I did

    sudo apt install qgis
    
and followed this [tutorial](https://wiki.openstreetmap.org/wiki/QGIS_tutorial) to open the .osm file. Finally, I installed a plugin called `Lat Lon Tools` and entered the coordinates to pinpoint the node with a red crosshair. It would be nice to just show the IDs right in QGIS to easily look at the other parts of the planned path but I didn't succeed.

Yes, it's the entrance to the parking spot that I selected in `rviz`
