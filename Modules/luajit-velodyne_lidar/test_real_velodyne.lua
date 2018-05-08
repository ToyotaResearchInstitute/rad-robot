#!/usr/bin/env luajit
local velodyne = require'velodyne_lidar'
local skt = require'skt'

print("Checking data", velodyne.DATA_PORT)

-- Check the data points
local skt_data = assert(skt.open{
  port = velodyne.DATA_PORT,
  use_connect = false
})

local data
while not data do
  local ret, ready = skt.poll({skt_data.fd}, 1e3)
  if ready then
    for _, i in ipairs(ready) do
      local pkt = skt_data:recv()
      data = velodyne.parse_data(pkt)
    end
  end
end

for k,v in pairs(data) do print(k, v) end

print()
print("Checking position", velodyne.POSITION_PORT)
-- Check optional GPS position
local skt_pos = assert(skt.open{
  port = velodyne.POSITION_PORT,
  use_connect = false
})

local pos
while not pos do
  local ret, ready = skt.poll({skt_pos.fd}, 1e3)
  if ready then
    for _, i in ipairs(ready) do
      local pkt = skt_pos:recv()
      pos = velodyne.parse_position(pkt)
    end
  end
end

for k,v in pairs(pos) do print(k, v) end