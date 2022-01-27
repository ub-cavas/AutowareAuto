Running Multi Object Tracker with the SVL Simulator {#running-tracker-with-vision}
===============================

# Setup the simulator
The object tracker in autoware is designed to subscribe to lidar cluster based objects and 2d detections from vision. To setup the simulator to do this change the sensor configuration of the vehicle. Go to the "Vehicles" page under the "Library" icon, select the desired vehicle and click the "Sensor Configuration" section icon on it. Click "Create New Configuration" button at the bottom of the page. Set a configuration name and pick "ROS2ForUnitySVLBridge" as a bridge. Click "{...}" symbol near Visual Editor window. Copy and paste the contents from [here](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/perception/segmentation/ground_truth_detections/config/lgsvl-sensors-camera.json) into the box. Create a simulation that uses this modified vehicle and a map of your choice. The current demo is designed to work on the AutonomouStuff parking lot map.

# Running the tracker
Go to the "Simulations" page, hit the "Run Simulation" button on the browser window to start the simulation that was created at the beginning.

Run the following launch command to launch the tracker,
```
ros2 launch autoware_demos object_tracker_lidar_single_camera_lgsvl.launch.py
```

This command by default uses the NDT localizer to get the ego vehicle state. But the NDT localizer requires a point cloud map which is available only for the `AutonomouStuff` map of the simulator. If you want to run the tracker in some other simulator map (for example, `BorregasAve`) run the following command to disable NDT,
```
ros2 launch autoware_demos object_tracker_lidar_single_camera_lgsvl.launch.py use_ndt:=False
```

Run the following launch command to launch the tracker with dual camera,
```
ros2 launch autoware_demos object_tracker_lidar_dual_camera_lgsvl.launch.py
```

Run the following launch command to launch the tracker with polygon prisms input and dual camera,
```
ros2 launch autoware_demos object_tracker_polygon_prisms_dual_camera_lgsvl.launch.py
```
