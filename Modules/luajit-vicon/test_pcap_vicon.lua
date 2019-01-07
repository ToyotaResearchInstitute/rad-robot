#!/usr/bin/env luajit
local pcap_name = assert(arg[1], 'Please provide a .pcap file')

local ffi = require'ffi'
local pcap = require'pcap'
local vicon = require'vicon'
local unpack = unpack or require'table'.unpack

local pcap_entries = assert(pcap.entries(pcap_name))
local OFFSET0 = 42

local t0
for t, pkt_n_len in pcap_entries do
  if not t0 then t0 = t end
  local pkt, len = unpack(pkt_n_len)
  pkt = pkt + OFFSET0
  len = len - OFFSET0
  -- Check that both methods work
  assert(vicon.parse(ffi.string(pkt, len)))
  local data = assert(vicon.parse(pkt, len))
  -- data = vicon.parse(pkt)
  print(string.format("== Frame %4d @ %.3f ==", data.frame, t-t0))
  for k,v in pairs(data) do
    if k~='frame' then
      print(string.format("ID [%s] {%.2f, %.2f, %.2f} {%.2f°, %.2f°, %.2f°}", k,
        v.translation[1], v.translation[2], v.translation[3],
        math.deg(v.rotation[1]), math.deg(v.rotation[2]), math.deg(v.rotation[3])))
    end
  end
end
