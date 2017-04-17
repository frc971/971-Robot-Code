#!/bin/bash

cat superstructure.cc | grep 'table entry' | sed 's/^.*{\(.*\), {\(.*\), \(.*\), \(.*\) \* M_PI.*$/\1, \2, \3, \4/' > /tmp/constants_plot

gnuplot plot_constants.gnuplot
