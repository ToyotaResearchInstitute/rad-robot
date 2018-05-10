#!/usr/bin/env luajit

-- AUTHOR: Stephen McGill, 2017
-- Usage: luajit test_pcap.lua FILENAME.pcap
-- Description: Iterate through packets from pcap log files

local pcap = require("pcap")

local filename = assert(arg[1], 'Please provide a filepath')
print("Opening", filename)
local it, hdr = pcap.entries(filename)
assert(it, hdr)
print("Header", unpack(hdr))
for t, entry in it do
  print(t, unpack(entry))
end
