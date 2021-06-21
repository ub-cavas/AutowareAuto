ground_truth_detections {#ground_truth_detections-package-design}
===========


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

The `ground_truth_detections` package converts the ground truth from the SVL
simulator regarding dynamic detections into the format understood by the
Autoware.Auto perception module.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->


## Assumptions / Known limits
<!-- Required -->

A camera sensor has to be added to the [sensor configuration of the vehicle in
SVL](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/lgsvl.html#lgsvl-configuring-vehicle)
and the ground-truth detections have to be output.

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
This node has no ROS2 parameters.

### Input

- **topic** `/simulator/ground_truth/detections2D`
- **type**  [`lgsvl_msgs::msg::Detection2DArray`](https://github.com/lgsvl/lgsvl_msgs/blob/master/msg/Detection2DArray.msg)

### Output

- **topic** `/simulator/ground_truth/detections2D_roi`
- **type**  [`autoware_auto_msgs::msg::ClassifiedRoiArray`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/ClassifiedRoiArray.idl)

## Inner-workings / Algorithms
<!-- If applicable -->

The labels of objects are character strings in SVL and can take arbitrary values. The following mapping to [`autoware_auto_msgs::msg::ObjectClassification`](https://gitlab.com/autowarefoundation/autoware.auto/autoware_auto_msgs/-/blob/master/autoware_auto_msgs/msg/ObjectClassification.idl) is performed:

| label in SVL  | object class |
|---------------|--------------|
| `Hatchback`   | `CAR`        |
| `Jeep`        | `CAR`        |
| `Sedan`       | `CAR`        |
| `SUV`         | `CAR`        |
| `BoxTruck`    | `TRUCK`      |
| `Pedestrian`  | `PEDESTRIAN` |
| anything else | `UNKNOWN`    |

The coordinates of bounding boxes in camera coordinates are transformed to polygons with four points with the first point being the lower left corner, the second being the lower right corner etc. The coordinates are guaranteed to be non-negative.

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

Not available.

# References / External links
<!-- Optional -->

- [SVL 2020.06 ground truth](https://www.svlsimulator.com/docs/archive/2020.06/perception-ground-truth/#subscribe-to-ground-truth-messages-from-simulator)
- [SVL 2021.01 ground truth 2D](https://www.svlsimulator.com/docs/user-interface/sensor-visualizers/#2d-ground-truth)
- [Visualize detections](https://www.svlsimulator.com/docs/archive/2020.06/perception-ground-truth/#view-estimated-detections-in-simulator) in SVL

# Future extensions / Unimplemented parts
<!-- Optional -->

1. Other detections like 3D detections or lidar ground truth

# Related issues
<!-- Required -->

- #1099 Initial implementation
