ssc_interface design {#ssc_interface-package-design}
====================


# Purpose / Use cases
This package acts as a message translator between AutonomouStuff's SSC software and Autoware.Auto.

# Design
This package inherits from @ref vehicle-interface-design and, as such, performs all of the same tasks but specifically with messages from the `automotive_autonomy_msgs` stack.

## Assumptions / Known limits
<!-- Required -->

## Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->


## Inner-workings / Algorithms
<!-- If applicable -->


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
- [Speed and Steering Control Software](https://autonomoustuff.com/product/astuff-speed-steering-control-software/)

# Future extensions / Unimplemented parts
<!-- Optional -->


# Related issues
- [#184](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/184)
