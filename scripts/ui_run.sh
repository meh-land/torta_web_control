#!/bin/bash

# Exit immediately if any error happens
set -e
#cd /gp/gp_ws/src/torta_pkg/scripts/
flask run --no-reload -h 0.0.0.0 &
# get IP address
my_ip="$(ifconfig enp0s3 |grep "inet " | awk '{print $2}')"
echo your IP is $my_ip
echo app runs on port 5000

# wait for user to kill app
echo Press Enter to kill the app
read temp

# Get PID of app and kill it
flask_pid="$(ps -a | grep flask | awk '{print $1}')"
kill -9 $flask_pid
echo ADIOS



set +e
