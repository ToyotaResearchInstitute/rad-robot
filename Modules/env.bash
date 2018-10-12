#!/bin/sh
export ROBOT_HOME=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
export LUA_PATH="$ROBOT_HOME/luajit-?/?.lua;$ROBOT_HOME/lua-?/?.lua;;"
export LUA_CPATH="$ROBOT_HOME/lua-?/?.so;;"
export LCM_DEFAULT_URL="udpm://239.255.65.56:6556?ttl=0"
# sudo sysctl -w net.core.rmem_max=2097152
# sudo sysctl -w net.core.rmem_default=2097152

# Set to 2^21 bytes
RMEM_MAX_LCM=2097152
RMEM_MAX=$(sysctl -n net.core.rmem_max)
test ! $RMEM_MAX -eq $RMEM_MAX_LCM && sudo sysctl -w net.core.rmem_max=$RMEM_MAX_LCM

# Set to 2^21 bytes
RMEM_DEFAULT_LCM=2097152
RMEM_DEFAULT=$(sysctl -n net.core.rmem_default)
test ! $RMEM_DEFAULT -eq $RMEM_DEFAULT_LCM && sudo sysctl -w net.core.rmem_default=$RMEM_DEFAULT_LCM

# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth1

# sudo apt-get install tmux nano htop libusb-1.0-0-dev

# sudo rmmod uvcvideo && sudo modprobe uvcvideo quirks=128
