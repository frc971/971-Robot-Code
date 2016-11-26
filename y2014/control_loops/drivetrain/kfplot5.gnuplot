set term x11

plot '/tmp/dump' using 1:($13 + $14 + 20) title 'dpower',\
     '/tmp/dump' using 1:(($9 + $10 - $11 - $12) * 10.0) title 'avg_velocity_error' with lines

pause -1
