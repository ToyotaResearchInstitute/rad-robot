#!/bin/sh
tcpdump -i en0 host 192.168.1.153 and port 51001 -n -s 0 -vvv -w ~/Desktop/rc.pcap
