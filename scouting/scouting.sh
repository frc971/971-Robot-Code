#!/bin/bash

# This script runs the webserver and asks it to host all the web pages.

exec \
    scouting/webserver/webserver_/webserver \
    -directory scouting/www/ \
    "$@"
