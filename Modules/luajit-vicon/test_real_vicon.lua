#!/usr/bin/env luajit
local vicon = require'vicon'
local skt = require'skt'

print("Listening to Vicon on", vicon.BASE_PORT)

-- Check the data points
local skt_vicon = assert(skt.open{
  port = vicon.BASE_PORT,
  use_connect = false
})

local t0 = os.time()
local data
while not data do
  local ret, ready = skt.poll({skt_vicon.fd}, 1e3)
  if ready then
    local pkt = skt_vicon:recv()
    data = vicon.parse(pkt)
  elseif not ret then
    os.exit(1)
  elseif os.difftime(t0, os.time()) > 10 then
    os.exit(1)
  end
end

for k,v in pairs(data) do
  print(k, v)
  if type(v)=='table' then
    for kk, vv in pairs(v) do
      if type(vv)=='table' then
        print(kk, table.concat(vv, ', '))
      else
        print(kk, vv)
      end
    end
  end
end
