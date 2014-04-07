#!/bin/bash

# Sets capabilities on a binary so it can run as realtime code as a user who
# doesn't have those rlimits or some other reason they can.
# Has to be run as root.

setcap 'CAP_IPC_LOCK+pie CAP_SYS_NICE+pie' $0

