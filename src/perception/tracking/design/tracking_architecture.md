Object tracking architecture {#tracking-architecture}
==============================

# Purpose / Use cases
This document defines the high level architecture that the object tracker follows.

# Architecture
![](images/tracking_architecture.png)

The core idea behind the design of this system is that we want to make use of multiple inputs into
the tracking pipeline. It is also generally hard to synchronize different inputs. The architecture is
guided by these two assumptions: it allows processing every message as early as it becomes
available, while still making use of other incoming messages.

@warning Due to the way messages are currently designed in Autoware.Auto, we use the
    `DetectedObjects` message for both clusters and machine-learning LiDAR inputs. This should not
    be the case. Clustering should output a shapeless blob of points (convex hull seems to be a good
    trade-off here), while the `DetectedObjects` message should have an explicit shape, like a box
    or a cylinder. This will allow to track the shape of the object and make use of other
    differences in these types. This is not implemented yet.

# Workflow
The fundamental operation of a tracker is as follows:

1. Associate the different perception inputs with the existing tracks
    1. Each input is transformed into a common frame of reference
    2. Tracks are predicted to the timestamp corresponding to the input
    3. Different association methods can be used for each input
2. Update tracks
    1. Each perception input could contribute different information to the track
    2. For example, LiDAR clusters provide track position, orientation and size update
    3. Image detections provide track track classification update
3. Track kinematic updates are carried out by EKF that uses Constant Acceleration motion model. Classification state of a track is also filtered with a specialized EKF
4. Create new tracks
    1. All the inputs that are not associated with any existing track are sent to the Track Creation module
    2. Based on the configured track creation policy new tracks are created and the inputs that are still unused are published as "Obstacles" (not tracks)
    3. One possible track creation policy is to create a track from every LiDAR cluster that has a corresponding image detection association. Image detections are generally not used for creating tracks since 3d position and orientation cannot be reliably estimated from image detection alone
5. Prune tracks that have not received an update for some time

# Description of submodules
This section will give a high level overview of each submodule mentioned in the workflow above. For a more detailed description refer to the design document specific to the submodule. The features described for each submodules may not be implemented yet. This serves as the guiding light.

## Association
### LiDAR clusters to Track association
Mahalanobis distance is used to compute a correspondence score between each track and detection.  
Gating based on area and euclidean distance is done to reduce the number of possible track-detection pairs for which the score has to be calculated.  
Once the scores are calculated assignment is carried out by Hungarian algorithm. The algorithm is modified so that it can handle cases where every detection-track pair may not have a valid score.  
More details in the [design doc](@ref tracking-detected-object-associator-design)

### Image detections to Track association
Tracks are projected on to the image plane.  
IOU is used as the correspondence score between a track and a detection.  
More details in the [design doc](@ref tracking-roi-associator)

## Vision - LiDAR cluster fusion
This section explains why we have two distinct blocks (ie., Image detection to track association and Cluster <--> Image association) instead of just one block (Cluster <--> Image association)

**Assumption:**  
Vision detections, in general, come in at a lower frequency than the LiDAR clusters   

LiDAR clusters are usually published at 10Hz. Vision detections from surround cameras are typically published at a lower rate. 

**Reason:**  
Based on the assumption, we will not have LiDAR clusters and vision detections looking at the same exact objects at the same exact time.  
Tracks have state information that make it easier to predict the position of the tracks at any give timestamp. This makes it easier and more accurate to associate tracks with vision detections.  
But not all tracks can be associated with vision detections. This is especially true for new objects that just arrived in the field of view.  
This means that we cannot ignore the unassociated vision detections. The best way to use them would be to associate them with LiDAR clusters.  
The lidar clusters will not be from the exact timestamp as the vision detections but we can increase the tolerances for the association. This enables us to create accurate and stable tracks for much longer range than if we just use LiDAR clusters.
