#!/usr/bin/env luajit

local hokuyo = require'hokuyo'
local SENSOR_IP = arg[1] or hokuyo.DEFAULT_IP
local SENSOR_PORT = hokuyo.DEFAULT_PORT

local version_pkt = assert(hokuyo.version())
local parameters_pkt = assert(hokuyo.parameters())
local status_pkt = assert(hokuyo.status())

local has_skt, skt = pcall(require, 'skt')
if not has_skt then
  print("No skt module for grabbing socket")
  os.exit()
end

print("\n== Connecting ==")
local transport = assert(skt.open{
  address = SENSOR_IP,
  port = SENSOR_PORT,
  tcp = true
})

print("\n== Stream Off ==")
for off in hokuyo.command_it(transport, "stream_off") do
  for k, v in pairs(off) do
    print(k, v)
  end
end

print("\n== Version ==")
for state in hokuyo.command_it(transport, "version") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Parameters ==")
for state in hokuyo.command_it(transport, "parameters") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Status ==")
for state in hokuyo.command_it(transport, "status") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

local function shutdown()
  print("\n== Stream Off ==")
  for off in hokuyo.command_it(transport, "stream_off") do
    for k, v in pairs(off) do
      print(k, v)
    end
  end
  transport:close()
  print("\n!! Exiting")
  os.exit()
end

local pkt_on = assert(hokuyo.stream_on())

print("\n== Stream On ==")
for state in hokuyo.command_it(transport, "stream_on") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Status ==")
for state in hokuyo.command_it(transport, "status") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Scans ==")
local co_scan = coroutine.create(hokuyo.update)

local pkt_continuous = assert(hokuyo.scan_continuous{
  intensity = true,
--  number = 3
})

local ret = transport:send(pkt_continuous)
-- Poll
n_pkt = 0
while skt.poll({transport.fd}, 50) > 0 do
  -- print("\n* DATA *")
  local response = transport:recv()
  -- io.write(response)
  -- print("Received", response and #response)
  local status, scan = coroutine.resume(co_scan, response)
  if not status then
    print("Coroutine", status, scan)
    break
  end
  while type(scan) == 'table' do
    n_pkt = n_pkt + 1
    if (n_pkt % 40) == 0 then print("Logged", n_pkt) end
    --[[
    print("\n!! Result")
    for k, v in pairs(scan) do
      if k~='scan' then
        print(k, v)
      else
        print("Scan: ", v and #v)
      end
    end
    --]]
    status, scan = coroutine.resume(co_scan)
    if not status then print("Coroutine", status, scan) end
  end
end

shutdown()
