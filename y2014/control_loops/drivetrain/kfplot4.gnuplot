set term x11

plot '/tmp/dump' using 1:4 title 'left_encoder',\
     '/tmp/dump' using 1:5 title 'right_encoder',\
     '/tmp/dump' using 1:15 title 'left_x',\
     '/tmp/dump' using 1:16 title 'right_x',\
     '/tmp/dump' using 1:6 title 'heading',\
     '/tmp/dump' using 1:18 title 'kf_heading',\
     '/tmp/dump' using 1:(($18 - $6) * 100) title 'kf_heading_error',\

pause -1
