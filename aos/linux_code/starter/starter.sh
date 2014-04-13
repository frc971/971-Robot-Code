#!/bin/bash

echo '/home/driver/tmp/robot_logs/%e-%s-%p-%t.coredump' > /proc/sys/kernel/core_pattern

chrt -o 0 bash -c "export PATH=$PATH:/home/driver/robot_code/bin; starter_loop.sh $*" &
STARTER_LOOP_PID=$!
echo Starter is ${STARTER_LOOP_PID}.
#chrt -o 0 bash -c "while true; do cd /home/driver/mjpg-streamer2; ./server.sh; sleep 5; done" &

# Log everything from the serial port...
#SERIAL_LOG_FILE=$(date "/home/driver/tmp/robot_logs/serial_log.%F_%H-%M-%S")
#chrt -o 0 bash -c "( stty -echo -echoe -echok 9600; cat > ${SERIAL_LOG_FILE} ) < /dev/ttyUSB0" &

# Wireshark _everything_ we can see...
#DUMPCAP_LOG_FILE=$(date "/home/driver/tmp/robot_logs/dumpcap.%F_%H-%M-%S")
#DUMPCAP_STDOUT_FILE=$(date "/home/driver/tmp/robot_logs/stdout_dumpcap.%F_%H-%M-%S")
#chrt -o 0 bash -c "dumpcap -i eth0 -w ${DUMPCAP_LOG_FILE} -f 'not port 8080 and not net 10.9.71.13' > ${DUMPCAP_STDOUT_FILE}" &

# Run netconsole to record what the cRIO sends.
chrt -o 0 bash -c '
NETCONSOLE_BASE=/home/driver/tmp/robot_logs/netconsole-
existing=$(ls ${NETCONSOLE_BASE}*)
if [[ $? -eq 0 ]]; then
	i=$(echo ${existing} | sed "s,${NETCONSOLE_BASE},,g; s/ /\n/g" | sort -g | tail -n1)
else
	i=0
fi
while true; do
	/home/driver/robot_code/bin/netconsole ${NETCONSOLE_BASE}$((++i))
	sleep 1
done
' &
NETCONSOLE_PID=$!
echo Netconsole is ${NETCONSOLE_PID}.

echo $$ > /tmp/starter.pid
