#set yrange [-13:2]
#set xrange [431.5:435.5]

set term x11

plot '/tmp/dump' using 1:9 title 'left_vel' with lines,\
     '/tmp/dump' using 1:10 title 'right_vel' with lines,\
     '/tmp/dump' using 1:(($9 + $10)/2) title 'avg_vel' with lines,\
     '/tmp/dump' using 1:20 title 'left_ss_vel' with lines,\
     '/tmp/dump' using 1:21 title 'right_ss_vel' with lines,\
     '/tmp/dump' using 1:11 title 'left_encoder_vel' with lines,\
     '/tmp/dump' using 1:12 title 'right_encoder_vel' with lines, \
     '/tmp/dump' using 1:(($11 + $12)/2) title 'avg_encoder_vel' with lines,\
     '/tmp/dump' using 1:($13 - 10) title 'left_voltage_error' with lines,\
     '/tmp/dump' using 1:($14 - 10) title 'right_voltage_error' with lines,\
     '/tmp/dump' using 1:(12 + $19 * 5) title 'browned_out' with lines

pause -1
