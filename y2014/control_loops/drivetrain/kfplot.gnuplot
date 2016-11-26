set term x11

plot '/tmp/dump' using 1:2 title 'left voltage' with lines,\
     '/tmp/dump' using 1:3 title 'right voltage' with lines,\
     '/tmp/dump' using 1:6 title 'gyro' with lines,\
     '/tmp/dump' using 1:7 title 'rate' with lines,\
     '/tmp/dump' using 1:8 title 'battery' with lines,\
     '/tmp/dump' using 1:9 title 'left_vel' with lines,\
     '/tmp/dump' using 1:10 title 'right_vel' with lines

pause -1
