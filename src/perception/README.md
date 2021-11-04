Perception {#autoware-perception-design}
==========

# Domain Description

The `perception` sub-directory is divided into three sections: `filters`, `segmentation`, and
`tracking`.

The `filters` directory contains nodes and libraries which are part of the filtering pipeline for 
sensor data. 

Nodes in this directory are able to work as standalone filtering modules or cascaded with other 
filtering algorithms working on the same sensor data types. 

Packages under the `segmentation` directory enable sensor data to be grouped into clusters or 
objects of interest which can provide downstream processes more information about the environment.

## Filters
- @subpage autoware-perception-filters-design

## Segmentation
- @subpage autoware-perception-segmentation-design

## Tracking
- @subpage tracking-architecture
- @subpage running-tracker-with-vision
- @subpage tracking-nodes-design
- @subpage tracking-detected-object-associator-design
- @subpage tracking-roi-associator
- @subpage tracking-track-creator
- @subpage tracking_test_framework-package-design
- @subpage projection
