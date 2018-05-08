#!/usr/bin/env luajit

-- AUTHOR: Stephen McGill, 2017
-- Usage: ./test_pcan FILENAME.pcap
-- Description: Decode CAN messages from ethernet frames captured with the PEAK CAN-Ethernet device

---------------
-- Dependencies
local ffi = require("ffi")
local pcap = require("pcap")
local pcan = require("pcan")
---------------

-- "tcpdump -i any" implies offset of 44, else 42
local OFFSET0 = 44
local PCAN_SZ = 36
local MAX_SZ = PCAN_SZ * 15

local ID_COUNT = {}
local SHOW_COUNT = os.getenv'SHOW_COUNT'

if SHOW_COUNT then
  io.stdout:write('ID', '\t', 'Count', '\t', 'name', '\n')
else
  io.stdout:write('t', '\t', 'ID', '\t', 'DLC', '\n')
end

local filename = assert(arg[1], 'Please provide a filepath')

for t, pkt_n_len in pcap.entries(filename) do
  local packet, len = unpack(pkt_n_len)
  packet = packet + OFFSET0
  len = len - OFFSET0
  for offset=0,len-1,PCAN_SZ do
    local id, msg, dlc = pcan.decode(packet + offset, PCAN_SZ)
    if dlc > PCAN_SZ then
      io.stderr:write(string.format(
        "Bad PCAN packet size: %d > %d\n", dlc, PCAN_SZ))
    else
      -- Print the ID and message length
      if not SHOW_COUNT then
        io.stdout:write(string.format('%f\t0x%03x\t%d\n', t, id, dlc))
      end
      local str = ffi.string(msg, dlc)
      if id<4096 then
        ID_COUNT[id] = (ID_COUNT[id] or 0) + 1
      else
        io.stderr:write(string.format("Bad ID [too big]: 0x%x\n", id))
      end
    end
    
  end
  
end

