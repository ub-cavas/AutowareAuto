autoware_auto_msg_transformations {#autoware_auto_msg_transformations-package-design}
===========

This is the design document for the `autoware_auto_msg_transformations` package.


# Purpose / Use cases
Many packages need to apply some geometric transformation to message types. The purpose of this package is to allow sharing this code, and to have a central resource for developers who want to learn how to transform some new message type.

# Design
At the core of this package are functions that transform a message type with a suitable `Eigen` type.

Why don't we extend the existing `tf2::doTransform` function template?

* `tf2::doTransform` takes a `TransformStamped` message as input, which often is not what the caller uses to represent the transformation.
* Internally, `tf2::doTransform` has to convert `TransformStamped` into a computation-friendly type. This causes overhead when `tf2::doTransform` is called many times with the same transform.
* `tf2::doTransform` usually uses the `KDL` geometry library, which not many people are familiar with compared to `Eigen`.
* Documentation on the available specialization is scattered, and sparse (e.g. it's unclear whether input and output may alias, or if any checks are performed).

## Assumptions / Known limits
It is assumed that all `Eigen::Isometry3X` objects are valid coordinate transformations, i.e. the linear part is a rotation matrix.

It is assumed that all messages are valid, e.g. covariance matrices are positive semidefinite and symmetrical.

## Inputs / Outputs / API
The package provides an overloaded function `change_frame(tf, msg)` that modifies `msg` in-place according to `tf`.

There is also a `change_frame_imm(tf, msg)` variant that does not operate in-place.

## Inner-workings / Algorithms
N/A

## Error detection and handling
N/A

# Security considerations
N/A

# References / External links
N/A

# Future extensions / Unimplemented parts
N/A

# Related issues
Issue 1106