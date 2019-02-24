#!/usr/bin/env ruby
#
#    Analyse robot log file to understand stilts lift
#        
#    The robot lift time was much longer than expected when it was
#    first tested.  This script reads the log file and creates plot
#    files and a gnuplot script for viewing the results.  From
#    viewing these results, it was determined that the lift
#    control loop profile acceleration needed to be increased
#    from 0.5 m/s^2 to 2.0 m/s^2.
#
#    Michael wrote this script and James and Austin helped
#    with understanding the results and selecting what to plot.
#    The script was written Feb 23, 2019 while sorting out
#    why the robot was lifting slowly.
#
#    The log files used with this script are at
#    https://robotics.mvla.net/frc971/2019/robotLogFiles/2019-02-23_MVHS_lab_lift_speed_debugging/
#
usage = <<EOF

USAGE: #{__FILE__} [-h] aos_log-number [[aos_log-number2] ...]

       -h   print this help message

     Processes a log file and create data plot files and a gnuplot script
     for viewing the results.  Be sure and uncompress the log files
     before processing them if needed.  i.e. gunzip aos_log-number.gz
     
     To easily view a subset of the log data,
     change the 'xrange' in the top of the aos_log-number.gnuplot file.
     View the plots with

       gnuplot aos_log-number.gnuplot

     Use the "q" key followed by "enter" to advance from one plot to
     the next.

     Click in the plot with the right mouse button to select the start
     of a region to zoom in on.  Move the mouse to the end of the region
     of interest and then right click again to complete the selection.
     Doing so on the first plot is a good way to look at a time slice
     of the log file.  Subsequent plots will have the same time range.
     This can also be set by updating the "set xrange" near the top of
     the aos_log-number file.
 
     The log files used with this script are at
     https://robotics.mvla.net/frc971/2019/robotLogFiles/2019-02-23_MVHS_lab_lift_speed_debugging/

EOF

# print the usage message if there are no arguements or the -h
# flag is given.
if ( (ARGV.length == 0) || (ARGV.length > 0 && ARGV[0].match(/^-*h/)) ) then
    puts usage
    exit
end

#
def run_cmd(cmd)
    puts "Running: "+cmd
    `tcsh -c "#{cmd}"`
end
#
puts "ARGV = #{ARGV.inspect}"
ARGV.each do |log_file|
puts "# Processing log file #{log_file}"

file_currents    = log_file + ".currents"    # PDP currents and battery voltage
file_voltages    = log_file + ".voltages"    # control loop output voltage
file_goal_status = log_file + ".goal_status" # status for stilts position, velocity,
                               # and control loop target speed and voltage error
file_goal_goal   = log_file + ".goal_goal"   # control loop target accleration
file_gnuplot     = log_file + ".gnuplot"     # gnuplot plot file.
file_debug       = log_file + ".debug"       # log_displayer debug level output

run_cmd("/home/michael/bin/log_displayer -l DEBUG #{log_file} > #{file_debug}")
run_cmd("grep currents #{file_debug} | sed -e 's/[,:\{\}]/ /g' -e 's/^.*at 000000//' -e 's/s//' > #{file_currents}")
run_cmd("egrep '^superstructure.*stilts_voltage' #{file_debug} | sed -e 's/[,:\{\}]/ /g' -e 's/^.*at 000000//' -e 's/s//'  -e 's/y2019.*stilts_voltage/stilts_voltage/' > #{file_voltages}")
run_cmd("egrep '^superstructure.*status' #{file_debug} | sed -e 's/[,:\{\}]/ /g' -e 's/^.*at 000000//' -e 's/s//'  -e 's/y2019.* stilts/stilts/' > #{file_goal_status}")
run_cmd("egrep '^superstructure.*goal:' #{file_debug} | sed -e 's/[,:\{\}]/ /g' -e 's/^.*at 000000//' -e 's/s//'  -e 's/y2019.* stilts/stilts/' > #{file_goal_goal}")

puts "Making the plot file #{file_gnuplot}"
plot_commands = <<EOF
#!/bin/env gnuplot
#
#     gnuplot script for viewing the results.  To easily view
#     a subset of the log data, change the 'xrange' in the top
#     of the aos_log-number.gnuplot file.  View the plots with
#
#       gnuplot #{file_gnuplot}
#
#     Use the "q" key followed by "enter" to advance from one plot to
#     the next.
#
#     Click in the plot with the right mouse button to select the start
#     of a region to zoom in on.  Move the mouse to the end of the region
#     of interest and then right click again to complete the selection.
#     Doing so on the first plot is a good way to look at a time slice
#     of the log file.  Subsequent plots will have the same time range.
#     This can also be set by updating the "set xrange" near the top of
#     the aos_log-number.gnuplot file.
#

# Set the terminal size for the plots in pixels.
set term qt size 1800, 900

# Set the default line with for the plot lines.
do for [i=1:8] { set linetype i linewidth 4 }

# Set the xrange for the plots to be the full length of the data.
# Edit this to limit the plot range.  i.e. use [9.71:17] to limit
# the plot time to 9.71 seconds to 17 seconds.
set xrange [*:*]

set yrange [*:*]
set y2tics
set ytics nomirror
set y2label "Voltage (Volts)"
set ylabel "Position (m)"
set xlabel "Time (seconds)"
set grid
volts = "#{file_voltages}"
currents = "#{file_currents}"
goal_status = "#{file_goal_status}"
goal_goal = "#{file_goal_goal}"

print ""
print "# Show the stilts position, motor voltage from the control loop,"
print "# and the battery voltage from the CAN power distribution board data."
print ""
print "# Enter a \\"q\\" key followed by \\"Enter\\" to advance to the next plot."
print ""
print "# Click in the plot with the right mouse button to select the start"
print "# of a region to zoom in on.  Move the mouse to the end of the region"
print "# of interest and then right click again to complete the selection."
print "# Doing so on the first plot is a good way to look at a time slice"
print "# of the log file.  Subsequent plots will have the same time range."
print "# This can also be set by updating the \\"set xrange\\" near the top of"
print "# the aos_log-number.gnuplot file."
print ""
#

set title "#{log_file}" noenhanced
plot \\
   goal_status using 1:14 title "Position" with lines, \\
   volts using 1:($6 > 0.4 || $6 < -0.4 ? $6 : 1/0) axis x1y2 with lines lw 2 tit "Motor Voltage", \\
   currents using 1:9 axis x1y2 tit "Battery Voltage" with lines
pause -1
set multiplot layout 4, 1 title "#{log_file}" noenhanced
unset title
set ylabel "Position (m)"
set xlabel "Time (seconds)"
unset y2label
unset y2tics
plot \
   goal_status using 1:14 title "Position" with lines

set ylabel "Voltage (Volts)"
set yrange [*:12.8]
plot \\
   volts using 1:($6 > 0.4 || $6 < -0.4 ? $6 : 1/0) with lines lw 2 tit "Motor Voltage", \\
   currents using 1:9 tit "Battery Voltage" with lines

set ylabel "Current (Amps)"
set yrange [*:*]
plot \\
   currents using 1:18 tit "Motor Current" with lines lw 2

set ylabel "Speed (meters/second)"
plot \\
  0.0 with lines lw 3 notitle, \\
  goal_status using 1:16 title "Control Loop Status Lift Speed" with lines lw 2, \\
  goal_status using 1:20 title "Control Loop Goal Lift Speed" with lines lw 2


unset multiplot
pause -1

Kv = 163.96 # rad/(volts*seconds)
Kt = 0.00530 # N*m/Amps
Resistance = 0.0896 # Ohms
pulley_radius = 0.0142 # Meters
gear_ratio = 30.95
omega(v,i) = Kv * ( v - i * Resistance )
speed(v,i) = pulley_radius * omega(v, i) / gear_ratio
set y2label "Voltage (Volts)"
set y2tics
set ytics nomirror

print "# The \\"Voltage and Current Lift Speed\\" line shows"
print "# the predicted speed of the lift using the measured"
print "# motor current and the battery voltage at the Power"
print "# Distribution Board.  It should match the \\"status"
print "# Control Loop Goal Lift Speed\\" line from the log"
print "# file.  It uses the equation:"
print "#     omega(v,i) = Kv * ( v - i * Resistance )"
print "#     speed(v,i) = pulley_radius * omega(v, i) / gear_ratio"
print ""

set title "#{log_file}" noenhanced
set ylabel "Lift Speed (meters/second)"
plot \\
  currents using 1:(v=$9, i=$18, speed(v,i)) tit "Voltage and Current Lift Speed" with lines lw 2, \\
  goal_status using 1:16 title "Control Loop Status Lift Speed" with lines lw 2, \\
  goal_status using 1:20 title "status Control Loop Goal Lift Speed" with lines lw 2, \\
  goal_goal using 1:12 title "max\\\\_speed Control Loop Goal Lift Speed" with lines lw 2, \\
  currents using 1:9 axis x1y2 tit "Battery Voltage" with lines lw 2, \\
  goal_status using 1:26 title "Control Loop Voltage Error" axis x1y2 with lines lw 2, \\
  volts using 1:($6 > 0.4 || $6 < -0.4 ? $6 : 1/0) axis x1y2 with lines lw 2 tit "Motor Voltage"
pause -1

set multiplot layout 3, 1 title "#{log_file}" noenhanced
unset title

set ylabel "Position (m)"
set y2label "Current (Amps)"
set y2range [*:*]
plot \\
   goal_status using 1:14 title "Position" with lines, \\
   currents using 1:18 axis x1y2 tit "Motor Current" with lines lw 2

set ylabel "Lift Speed Acceleration (meters/second^2)"
unset y2label
unset y2tics
set ytics mirror
plot \\
  goal_goal using 1:14 title "Control Loop Lift Speed max\\\\_acceleraton Goal" with lines lw 2

set y2label "Voltage (Volts)"
set y2tics
set ytics nomirror

set ylabel "Lift Speed (meters/second)"
plot \\
  currents using 1:(v=$9, i=$18, speed(v,i)) tit "Voltage and Current Lift Speed" with lines lw 2, \\
  goal_status using 1:16 title "Control Loop Status Lift Speed" with lines lw 2, \\
  goal_status using 1:20 title "Control Loop Goal Lift Speed" with lines lw 2, \\
  goal_goal using 1:12 title "max\\\\_speed Control Loop Goal Lift Speed" with lines lw 2, \\
  currents using 1:9 axis x1y2 tit "Battery Voltage" with lines lw 2, \\
  goal_status using 1:26 title "Control Loop Voltage Error" axis x1y2 with lines lw 2, \\
  volts using 1:($6 > 0.4 || $6 < -0.4 ? $6 : 1/0) axis x1y2 with lines lw 2 tit "Motor Voltage"
unset multiplot

pause -1
EOF

File.open(file_gnuplot, 'w') { |f| f.write(plot_commands) }

puts <<EOF

Done making data files and plot file.

View the plots with

   gnuplot #{file_gnuplot}

Use the "q" key followed by "enter" to advance from one
plot to the next.

EOF

end
