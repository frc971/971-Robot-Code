#!/bin/bash

for ((i=1; 1; i++)); do
	starter_exe $* 1>/tmp/starter${i}_stdout 2>/tmp/starter${i}_stderr
	sleep 2
done
