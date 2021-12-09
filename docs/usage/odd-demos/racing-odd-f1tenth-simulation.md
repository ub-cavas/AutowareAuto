F1Tenth Demonstration {#f1tenth-demo}
=================================================

@tableofcontents

# Setup Simulation {#f1tenth-simulation}

## Launching the simulator
Launch the simulator by
```
$ cd ~/adehome/AutowareAuto

# If you are using a joystick, add `-- --device /dev/input/js0`
$ ade --rc .aderc-amd64-foxy-lgsvl start --update --enter

ade$ /opt/lgsvl/simulator
```

If you have never setup LGSVL before, please follow the instructions in @ref lgsvl page until the section called “Configure the cluster”.

## Creating a simulation
### Choosing a map
The goal is to add `Red Bull Ring Racetrack` map to your library. If that map is already in your library then nothing needs to be done.

Adding a map to your library:
- Go to `Store` -> `Maps`.
- Click `+` button next to `Red Bull Ring Racetrack` map (you can use search bar to filter by name).

@image html images/f1tenth-svl-map.png "Choosing a map" width=50%

### Configuring a vehicle {#lgsvl-configuring-vehicle}

The goal is to add `F1TenthCar` vehicle to your library. If this vehicle is already in your library then nothing needs to be done.

Adding a vehicle to your library:
- Go to `Store` -> `Vehicles`.
- Click `+` button next to `F1TenthCar` vehicle (you can use search bar to filter by name).

@image html images/f1tenth-svl-vehicle.png `Adding a vehicle` width=50%

### Configure vehicle sensors

Once you added vehicle to your library:
- Go to `Library` -> `Vehicles`.
- Click on the `F1TenthCar` portrait. You will be forwarded to a vehicle edit page.
- Click a button near `Sensor Configurations` section to modify sensor configurations.

Create a new one and use the most recent version of sensor configuration file:
- Click `+ Create New Configuration` button at the bottom of the page.
- Set a configuration name and pick `ROS2` as a bridge. - Confirm.

In the configuration edit view:
- Click `{...}` symbol near Visual Editor (preview) window.
- Paste contents of [avp-sensors.json](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/f1tenth-devel/src/launch/autoware_demos/config/svl/fitenth.json) file inside edit window. `Configurator` window should populate with bunch of sensors now.
- Click `Save` to save configuration.

@image html images/f1tenth-svl-sensors-json.png "Json configuration file" width=40%

### Choosing/creating a simulation

The SVL simulator lets you store and reuse multiple simulation configurations. To use an existing simulation, navigate to `Simulations` tab and press the "Run Simulation" button for desired instance. The simulator should now start in the SVL window.

To create a new simulation, follow the below steps:

- Switch to the `Simulations` tab and click the `Add new` button.
- Enter a name, description and select a cluster. Click `Next`.
- Select the `Random Traffic` runtime template from the drop-down menu.
- Select `Red Bull Ring Racetrack` map and `F1TenthCar` vehicle with your sensor configuration. Click `Next`.
- Select `Autoware.Auto` autopilot and leave `Bridge Connection` with default value.
- Click `Next` and then `Publish`.

You can visit [SVL documentation](https://www.svlsimulator.com/docs/user-interface/web/simulations/) for more in-depth description.

# F1Tenth RecordReplay Trajectory Demo

## Starting ade and setting up Autoware for F1Tenth
F1Tenth is currently developed under the `f1tenth-devel` branch of Autoware.Auto. This step won’t be necessary once the [issue](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/1317) is resolved.

```{bash}

# start ade
# If you are using a joystick, add `-- --device /dev/input/js0`
$ cd adehome/AutowareAuto
$ ade --rc .aderc-amd64-foxy-lgsvl start --update --enter

# build autoware
ade$ cd AutowareAuto
ade$ git checkout f1tenth-devel
ade$ vcs import . < autoware.auto.foxy.repos
ade$ sudo apt update; rosdep update; rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -yr
ade$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Launching simulator
```{bash}
$ ade enter
$ /opt/lgsvl/simulator
```

In your browser, start the Simulation environment you setup in the previous section.

## Creating Map
Skip this step if you want to use redbull_ring_racetrack. The map is already provided.

```{bash}
# (Terminal 1)
$ ade enter
ade$ source AutowareAuto/install/setup.bash

# Optional arguments:
#   `with_joy:=True` to drive vehicle using joystick
ade$ ros2 launch autoware_demos f1tenth_mapping_demo.launch.py
```

@image html images/f1tenth-mapping.png "F1Tenth mapping file" width=40%

In LGSVL, drive around the vehicle. Make sure to drive more than one complete lap for the mapping node to detect loop closure.

Save the map by running:
```{bash}
# (Terminal 2)
ade$ mkdir ${HOME}/map
ade$ ros2 run nav2_map_server map_saver_cli -f $HOME/map/redbull_ring_racetrack
```

After saving maps, stop mapping nodes by pressing `Ctrl+C`

## Recording and replaying trajectory
### Record a trajectory

```{bash}
# (Terminal 1)
$ ade enter
ade$ source AutowareAuto/install/setup.bash

# Optional arguments:
#   `with_joy:=True` to drive vehicle using joystick
#   `map:=/path/to/map.yaml` to select your original map

ade$ ros2 launch autoware_demos f1tenth_recordreplay_demo.launch.py
```

Set a intial pose with correct orientation in Rviz using `2D pose estimate`

```
# (Terminal 2)
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/recordtrajectory autoware_auto_planning_msgs/action/RecordTrajectory "{record_path: "/tmp/path"}" --feedback
```
In LGSVL, drive around the vehicle and stop recording with Ctrl + C in terminal where `f1tenth_recordreplay_demo.launch.py` was launched.

### Replay a trajectory

```{bash}
(Terminal 1)
ade$ source /opt/AutowareAuto/setup.bash

# Optional arguments:
#   `map:=/path/to/map.yaml` to select your original map
ade$ ros2 launch autoware_demos f1tenth_recordreplay_demo.launch.py
```

Set a intial pose with correct orientation in Rviz using `2D pose estimate`

```{bash}
(Terminal 2)
ade$ source /opt/AutowareAuto/setup.bash
ade$ ros2 action send_goal /planning/replaytrajectory autoware_auto_planning_msgs/action/ReplayTrajectory "{replay_path: "/tmp/path"}" --feedback
```

@image html images/f1tenth-replay-path.png "F1Tenth replay trajectory file" width=40%

## Known issues
The replayed velocity might not be exactly the same with the recorded one. This is due to limitation of `Pure Pursuit` algorithm. The implementation does not take delay and external force into consideration, which means that it assumes constant speed when acceleration is released even if break is not pressed. This causes velocity of the vehicle to be wrong. Improvements will be made in the future.