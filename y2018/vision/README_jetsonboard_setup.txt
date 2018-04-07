# Put the following lines in the file shown.

echo "Current contents are: "
cat  /etc/supervisor/conf.d/vision.conf 
cat > /etc/supervisor/conf.d/vision.conf <<EOF
[program:vision]
command=/root/startup.sh
autorestart=true
startsecs=3
startretries=200
redirect_stderr=true
EOF


# before we switched at svr at 9:47 am, the cameras were.
# The left camera is right side up and red
# The right camera is upside down and green

# before we switched at svr at 9:47 am, the cameras were.
# The left camera is green
# The right camera is red
