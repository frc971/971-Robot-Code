#!/bin/bash -x

egrep 'Via|Pin' $1 | wc -l
