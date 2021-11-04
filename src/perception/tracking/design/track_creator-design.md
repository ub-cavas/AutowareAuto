Track Creator {#tracking-track-creator}
===================

# Purpose / Use cases

Object tracking for an autonomous driving system involves associating new detections in every frame to tracks that the tracker keeps account of. New objects enter the field of view of the vehicle every frame and the perception sensor may also have false positives every frame. This means the tracker needs a module that will assess these detections that do not meet the matching criteria to be matched with any of the existing tracks. This module ideally needs to create new tracks only for detections that are from real objects which the tracker can take into its operation. 

# Design

The core idea behind track creation is that we use a strategy pattern, where most of the work for
track creation is actually taken care of by a track creation policy. The
autoware::perception::tracking::TrackCreator class takes the policy as a template parameter and
calls it upon receiving data used for track creation.

@note While this is a polymorphic behavior, we use compile-time polymorphism, so the validity of the
    types of inputs to the track creation module is validated during the compilation.

## Inner-workings / Algorithms

The autoware::perception::tracking::TrackCreator class does not do much work on its own, but defers
it to one of the policies described below:

### LidarClusterOnly
- Call `create_tracks(const ObjectsWithAssociations & objects)`. This method will create one track per provided cluster. Only clusters that are not associated to anything else is considered here.
- Using this policy does not require a valid `GreedyRoiAssociator` object pointer. It can be initialized to nullptr
- This policy will not implement the `add_objects()` function for `ClassifiedROIArray` type message. Calling that will result in an exception being thrown

### LidarClusterIfVision
- Call `add_objects()` with the vision detections message and result from the vision-track association. This method will go through every vision detection that was not associated to a track and store it internally. Every time this function is called a new vision detection message is created internally and pushed to a cache 
- Call `create_tracks(const ObjectsWithAssociations & objects)`. This method will first try to find a vision detection that is within `max_vision_lidar_timestamp_diff` from the LiDAR cluster msg stamp. If it finds such a message it will try to associate the LiDAR clusters and the vision detections. New tracks will be created only from LiDAR clusters that are matched with a vision detection
- Using this policy requires a valid `VisionPolicyConfig` struct object to be initialized in the `TrackCreatorConfig` struct object. 

## Parameters
- TrackCreationPolicy - Choose one of the available policies based on the above explanation
- Default variance & Noise variance - These are values to be used in the initialization of new [TrackedObject](@ref autoware::perception::tracking::TrackedObject) objects
- VisionPolicyConfig - This struct needs to be initialized for the LidarClusterIfVision policy
  - Transform from base_link to camera
  - iou_threshold - Minimum IOU value to math a LiDAR cluster and a vision detection
  - Instrinsic parameters of the camera
  - max_vision_lidar_timestamp_diff - Maximum difference between vision detections message stamp and LiDAR clusters message stamp to be considered for association

## Assumptions / Known Limitations
- This module assumes `create_tracks` will be called every time a LiDAR objects message is received
- All the functions in the module are assumed to be called from the same thread except for `add_objects(ClassifiedROIArray, ...)` which uses thread-safe data structures and can be called from a different thread 

# Future extensions / Unimplemented parts
- Support for radar detections
- Support for machine-learning LiDAR object detections
