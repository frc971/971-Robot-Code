#!/usr/bin/gnuplot

n=1000 #number of intervals
max=500. #max value
min=0 #min value
width=(max-min)/n #interval width
#function used to map a value to the intervals
hist(x,width)=width*floor(x/width)+width/2.0
set boxwidth width*0.9
set style fill solid 0.5 # fill style

set logscale y
set yrange [0.1:]
set xrange [0:200]

#count and plot
plot "/tmp/st_csv" u (hist($1,width)):(1.0) smooth freq w boxes, \
     "/tmp/pt_csv" u (hist($1,width)):(1.0) smooth freq w boxes, \
     "/tmp/ft_csv" u (hist($1,width)):(1.0) smooth freq w boxes

pause -1

