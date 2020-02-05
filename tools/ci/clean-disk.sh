#!/bin/bash

#Retrieve Jenkins node name or set a test value
NODE_NAME=${NODE_NAME:-"TEST971"}
#Set default for disk utilisation
DMAX=${1:-80%}
#Retrieve disk usages in percentage
DSIZE=$(df -hlP /home/jenkins | sed 1d | awk '{print $5}')

if [[ $DSIZE>$DMAX ]]; then
		echo $NODE_NAME": Disk over "$DMAX" Clean up needed on node."
		rm -rf ~jenkins/.cache/bazel/disk_cache;
else
    echo $NODE_NAME": No clean up needed. Disk usage is at: "$DSIZE
fi
