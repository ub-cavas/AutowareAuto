global_velocity_planner {#global_velocity_planner-package-design}
===========

This is the design document for the `global_velocity_planner` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->
It creates trajectories. It will make easier to test controllers. It provides additional information
like the reference velocity and acceleration for the controllers.



# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->
It works like lane_planner, it takes the route from lanelet2_global_planner, then it constitutes the
waypoints. By using these waypoints, it calculates the curvatures of the corresponding waypoints.
After these calculations, it fills longitudinal velocities of these waypoints with respect to 
maximum lateral and longitudinal acceleration which are taken from parameter file.

## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
Inputs:
* Global route

Parameters:
* Longitudinal acceleration
* Lateral acceleration
* Trajectory resolution
* cg_to_front_m
* cg_to_rear_m

Outputs:
* Trajectory


## Inner-workings / Algorithms
<!-- If applicable -->
1. Generation of trajectory points from centerline in Lanelet Map
2. Calculation of curvatures of the trajectory points.
3. Calculation of longitudinal velocities with respect to lateral and longitudinal accelerations.
4. Calculation of steering angle and orientation of each trajectory point.
5. Calculation of time of arrival for each points from velocity.
6. Resizing the trajectory.


## Error detection and handling
<!-- Required -->


# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->
* Steering angle for first point will be implemented.
* get_closest_index function will be optimized.

# Related issues
<!-- Required -->
