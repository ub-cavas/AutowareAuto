child_frame_transformer_nodes {#child_frame_transformer_nodes-package-design}
===========

This is the design document for the `child_frame_transformer_nodes` package.


# Purpose / Use cases
This package allows the creation of simple nodes which can transform a provided measurement.
The transform can be applied to either the parent frame of the measurement or the child frame.

# Design

## Assumptions / Known limits
Only works with measurement types which have a `doTransform` function defined in `tf2_geometry_msgs`.

## Inputs / Outputs / API
A node which inherits from @ref autoware::measurement_transformer_nodes::ChildFrameTransformerNode requires the following parameters:

- `input_child_frame`
- `output_child_frame`

A node which inherits from @ref autoware::measurement_transformer_nodes::ParentFrameTransformerNode requires the following parameters:

- `output_parent_frame`

All nodes subscribe to a measurement topic and publish to a different measurement topic. These
are different per node type.

For @ref autoware::measurement_transformer_nodes::PoseChildFrameTransformerNode, the defaults are:

- `pose_in` (input topic)
- `pose_out` (output topic)

## Inner-workings / Algorithms
Transforms of the parent frame of a measurement are straight-forward.
A `tf2::TransformListener` looks up the transform between the provided output frame and the
`frame_id` in the header of the received measurement. If the transform is available, it is
applied to the measurement and the resulting transformed measurement is published.

Transforms of the child frame are more complex. If both parent and child frames are fixed to
the same rigid body then the transform is straight-forward. However, we cannot assume this to
be true and if the parent and child frames are not fixed to the same rigid body, applying a
transform between two child frames to the measurement in a parent frame results in an incorrect
measurement.

An example: If you have two fixed frame origins on a vehicle and you receive a measurement of
one fixed frame (let's say `/base_link`) in some external parent frame (let's say `/odom`) which is
not affixed to the vehicle. If you then want to change the child frame to another fixed frame
on the vehicle (for example, `/nav_base`), you would start by looking up the transform between
the two vehicle-fixed frames. However, because the received transform represents the translation
and rotation of one child frame *referenced from the other child frame*, applying this transform
to a measurement taken in the parent frame results in an incorrect measurement. To work around
this, the @ref autoware::measurement_transformer_nodes::ChildFrameTransformerNode performs the following steps for each measurement:

- Converts the received measurement into a transform between the parent frame and the
  origin child frame
- Looks up the transform between the origin child frame and the target child frame
- Converts the child-frame transform into a measurement
- Applies the parent-to-child-measurement transform to the child-frame-transform measurement

The result of this set of steps is a correct measurement of the new child frame in the original
parent frame.


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
Make applicable to more message types.


# Related issues
<!-- Required -->
