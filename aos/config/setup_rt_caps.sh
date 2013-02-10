#!/bin/bash
# has to be run as root

setcap 'CAP_IPC_LOCK+pie CAP_SYS_NICE+pie' $0

