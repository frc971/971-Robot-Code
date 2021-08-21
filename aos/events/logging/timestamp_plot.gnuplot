#!/usr/bin/gnuplot -c

set format y "%.6f";
set mouse mouseformat "%.6f, %.9f"

node1 = ARG1
node2 = ARG2

print "Node1: ", node1
print "Node2: ", node2

node1_index = int(system("grep -n " . node1 . " /tmp/timestamp_noncausal_starttime.csv | sed 's/:.*//'")) + 1
node2_index = int(system("grep -n " . node2 . " /tmp/timestamp_noncausal_starttime.csv | sed 's/:.*//'")) + 1

noncausalfile12 = sprintf("/tmp/timestamp_noncausal_%s_%s.csv", node1, node2)
noncausalfile21 = sprintf("/tmp/timestamp_noncausal_%s_%s.csv", node2, node1)

samplefile12 = sprintf("/tmp/timestamp_noncausal_%s_%s_samples.csv", node1, node2)
samplefile21 = sprintf("/tmp/timestamp_noncausal_%s_%s_samples.csv", node2, node1)

offsetfile = "/tmp/timestamp_noncausal_offsets.csv"

#set term qt 0
if (ARG3 ne "" ) {
     set term png
     set output ARG3
}

plot samplefile12 using 1:2 title 'sample 1-2', \
     samplefile21 using 1:(-$2) title 'sample 2-1', \
     noncausalfile12 using 1:3 title 'nc 1-2' with lines, \
     noncausalfile21 using 1:(-$3) title 'nc 2-1' with lines, \
     offsetfile using 1:(column(node2_index) - column(node1_index)) title 'filter 2-1' with linespoints

if (ARG3 ne "" ) {
     exit
}

pause -1
