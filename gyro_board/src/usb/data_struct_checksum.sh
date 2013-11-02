#!/bin/bash

sum $(dirname $0)/data_struct.h | sed 's/^\([0-9]*\) .*$/\1/'
