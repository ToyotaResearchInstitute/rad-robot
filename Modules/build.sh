#!/bin/sh
test -d "$ROBOT_HOME" && \
find "$ROBOT_HOME" -maxdepth 2 -type f -name Makefile -execdir make \;
