#!/bin/sh
test ! -d "$ROBOT_HOME" && echo "No Robot" && exit
find "$ROBOT_HOME" -maxdepth 2 -type f -name Makefile -execdir make \;
