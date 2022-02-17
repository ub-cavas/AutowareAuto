Freespace planner {#freespace-planner}
===============

This is the design document for the `freespace_planner` package.

# Purpose / Use cases

Freespace planner provides implementations of different searching algorithms.
Package implements base class from which each implementation inherits.
Each implementation generates smooth paths from start pose to goal pose respecting kinematic constrains of the vehicle.

Currently implemented algorithms:

* Hybrid A*

# Design

Planner needs a representation of an environment in order to plan a trajectory.
It is provided in form of [nav_msgs::msg::OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html) costmap.
Algorithm outputs custom `PlannerWaypoints` object which definition can be found in code docs.

Having the costmap algorithms can create smooth and kinematically feasible trajectories that avoid obstacles.

Additionally the algorithm has implemented the Reeds-Shepp cost estimation algorithm, which makes the found paths smooth and optimal.

Planning returns a boolean that indicates if planning succeeded and one of the following statuses for better verbosity:
* `SUCCESS` - planning succeeded
* `FAILURE_COLLISION_AT_START` - planning failed because of an obstacle inside vehicle's footprint at the starting
  position
* `FAILURE_COLLISION_AT_GOAL` - planning failed because of an obstacle inside vehicle's footprint at the goal position
* `FAILURE_TIMEOUT_EXCEEDED` - planning failed because timeout has been exceeded
* `FAILURE_NO_PATH_FOUND` - planning failed because no smooth and kinematically feasible trajectory could be found

Planning does not return the planned trajectory, but it can be accessed by the appropriate getter.

## Configuration

### Common planner parameters

| Parameter                     | Type   | Unit | Description                                                                                    |
| ----------------------------- | ------ | ---- | ---------------------------------------------------------------------------------------------- |
| `time_limit`                  | double | ms   | time limit of planning                                                                         |
| `robot_length`                | double | m    | robot length                                                                                   |
| `robot_width`                 | double | m    | robot width                                                                                    |
| `minimum_turning_radius`      | double | m    | minimum turning radius of robot                                                                |
| `theta_size`                  | double | -    | the number of angle's discretization                                                           |
| `goal_lateral_tolerance`      | double | m    | lateral tolerance of goal pose                                                                 |
| `goal_longitudinal_tolerance` | double | m    | longitudinal tolerance of goal pose                                                            |
| `goal_angular_tolerance`      | double | rad  | angular tolerance of goal pose                                                                 |
| `curve_weight`                | double | -    | additional cost factor for curve actions                                                       |
| `reverse_weight`              | double | -    | additional cost factor increasing trajectory cost when changing <br> move direction to reverse |
| `obstacle_threshold`          | double | -    | threshold for regarding a certain grid cell as obstacle                                        |

### Hybrid A* parameters

| Parameter                     | Type   | Unit | Description                                             |
| ----------------------------- | ------ | ---- | ------------------------------------------------------- |
| `use_back`                    | bool   | -    | whether using backward trajectory                       |
| `use_reeds_shepp`             | bool   | -    | whether using Reeds-Shepp cost estimation algorithm     |
| `only_behind_solutions`       | bool   | -    | whether restricting the solutions to be behind the goal |
| `distance_heuristic_weight`   | double | -    | heuristic weight for estimating node's cost             |

# References / External Links

[Hybrid A* paper](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)

[Reeds-Shepp paper](https://projecteuclid.org/journals/pacific-journal-of-mathematics/volume-145/issue-2/Optimal-paths-for-a-car-that-goes-both-forwards-and/pjm/1102645450.full?tab=ArticleLink)
