#set yrange [-15:15]

set term x11

plot '/tmp/dump' using 1:($13 - $14) title 'dpower' with lines,\
     '/tmp/dump' using 1:($9 - $10 - ($11 - $12)) title 'turn_vel_error' with lines,\
     '/tmp/dump' using 1:($9 - $10) title 'turn_vel' with lines,\
     '/tmp/dump' using 1:(($9 - $10) * ($13 - $14)) title 'correlation' with lines
