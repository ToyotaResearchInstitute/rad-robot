#!/bin/sh
export ROBOT_HOME=$(realpath "$(dirname "${BASH_SOURCE[0]}")")
export LUA_PATH="$ROBOT_HOME/luajit-?/?.lua;$ROBOT_HOME/lua-?/?.lua;;"
export LUA_CPATH="$ROBOT_HOME/lua-?/?.so;;"
export LCM_DEFAULT_URL="udpm://239.255.65.56:6556?ttl=0"
# sudo sysctl -w net.core.rmem_max=2097152
# sudo sysctl -w net.core.rmem_default=2097152

# Set to 2^21 bytes
#RMEM_MAX_LCM=2097152
# Via DALSA
RMEM_MAX_LCM=33554432
RMEM_MAX=$(sysctl -n net.core.rmem_max)
test ! $RMEM_MAX -eq $RMEM_MAX_LCM && sudo sysctl -w net.core.rmem_max=$RMEM_MAX_LCM

# Set to 2^21 bytes
#RMEM_DEFAULT_LCM=2097152
# Via DALSA
RMEM_DEFAULT_LCM=33554432
RMEM_DEFAULT=$(sysctl -n net.core.rmem_default)
test ! $RMEM_DEFAULT -eq $RMEM_DEFAULT_LCM && sudo sysctl -w net.core.rmem_default=$RMEM_DEFAULT_LCM

# Via DALSA
RMEM_MIN_LCM=12288
RMEM_MIN=$(sysctl -n net.ipv4.udp_rmem_min)
test ! $RMEM_MIN -eq $RMEM_MIN_LCM && sudo sysctl -w net.ipv4.udp_rmem_min=$RMEM_MIN_LCM

# Via DALSA
QLEN_LCM=7384
QLEN=$(sysctl -n net.unix.max_dgram_qlen)
test ! $QLEN -eq $QLEN_LCM && sudo sysctl -w net.unix.max_dgram_qlen=$QLEN_LCM

# Via DALSA
BACKLOG_LCM=4096
BACKLOG=$(sysctl -n net.core.netdev_max_backlog)
test ! $BACKLOG -eq $BACKLOG_LCM && sudo sysctl -w net.core.netdev_max_backlog=$BACKLOG_LCM

# sudo sysctl -w net.core.netdev_budget=600

# sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev eth1

# sudo apt-get install tmux nano htop libusb-1.0-0-dev libproj-dev

# sudo rmmod uvcvideo && sudo modprobe uvcvideo quirks=128

