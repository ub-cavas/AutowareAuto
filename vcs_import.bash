#!/bin/bash

SCRIPT=$(readlink -f $0)
SCRIPTPATH=`dirname $SCRIPT`

vcs import $SCRIPTPATH < $SCRIPTPATH/autoware.auto.foxy.repos
rm -r $SCRIPTPATH/src/external/perception_pcl/pcl_conversions
