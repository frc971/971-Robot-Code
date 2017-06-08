plot '/tmp/constants_plot' using 1:(($2 - 0.3) * 1000) title 'hood' with linespoints, \
     '/tmp/constants_plot' using 1:(($3 - 300) * 10) title 'flywheel' with linespoints, \
     '/tmp/constants_plot' using 1:($4 * 100) title 'indexer with linespoints

pause -1
