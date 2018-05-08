#!/bin/sh
test -d "$RACECAR_HOME" && \
find "$RACECAR_HOME" -maxdepth 2 -type f -name Makefile -execdir make clean \;