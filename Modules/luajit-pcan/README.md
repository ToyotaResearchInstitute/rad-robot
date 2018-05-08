# luajit-pcan
Decode CAN messages from ethernet frames captured with the 
[PEAK CAN-Ethernet device](http://www.peak-system.com/PCAN-Ethernet-Gateway-DR.330.0.html?&L=1)

## Installation

Ensure [LuaJIT 2.1](http://luajit.org/) is installed.

Install:
```sh
luarocks make
```

## Usage

Test on a *.pcap logfile

```sh
luajit test_pcan.lua FILE.pcap
```

### Show some statistics

Sort by CAN ID

```sh
env SHOW_COUNT=1 luajit test_pcan.lua FILE.pcap | sort -nk1
```

Sort by number of messages

```sh
env SHOW_COUNT=1 luajit test_pcan.lua FILE.pcap | sort -nk2
```
