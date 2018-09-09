#!/usr/bin/gnuplot

plot '/tmp/50' using 1:($2 / 10.0) title 'current0' lc 1 lt 'dashed' with lines, \
     '/tmp/50' using 1:($3 / 10.0) title 'current1' lc 2 lt 'dashed' with lines, \
     '/tmp/50' using 1:($4 / 10.0) title 'current2' lc 3 lt 'dashed' with lines, \
     '/tmp/50' using 1:(($5 * 0.666 - $6 * 0.333 - $7 * 0.333) / 10.0) title 'duty0' lc 1 lt 2, \
     '/tmp/50' using 1:((-$5 * 0.333 + $6 * 0.666 - $7 * 0.333) / 10.0) title 'duty1' lc 2 lt 2, \
     '/tmp/50' using 1:((-$5 * 0.333 - $6 * 0.333 + $7 * 0.666) / 10.0) title 'duty2' lc 3 lt 2, \
     '/tmp/50' using 1:($8 / 2048.0 * 10.0 * 6.28) title 'angle', \
     '/tmp/50' using 1:($9 / 1000.0 * 10.0) title 'velocity', \
     '/tmp/50' using 1:($10 / 10.0) title 'goal_current0' lc 1 with lines, \
     '/tmp/50' using 1:($11 / 10.0) title 'goal_current1' lc 2 with lines, \
     '/tmp/50' using 1:($12 / 10.0) title 'goal_current2' lc 3 with lines

     #'/tmp/50' using 1:($5 / 3000.0 * 100.0) title 'duty0' lc 1 lt 2, \
     #'/tmp/50' using 1:($6 / 3000.0 * 100.0) title 'duty1' lc 2 lt 2, \
     #'/tmp/50' using 1:($7 / 3000.0 * 100.0) title 'duty2' lc 3 lt 2, \

pause -1
