#!/usr/bin/env luajit

local hokuyo = require'hokuyo'
local SENSOR_IP = arg[1] or hokuyo.DEFAULT_IP

-- Form packets
local pkt_version = assert(hokuyo.version("hello world"))
local pkt_parameters = assert(hokuyo.parameters())
local pkt_status = assert(hokuyo.status())
local pkt_scan_single = assert(hokuyo.scan_single{
  -- str = "okie dokie"
})
--
local pkt_on = assert(hokuyo.stream_on())
local pkt_off = assert(hokuyo.stream_off())
local pkt_continuous = assert(hokuyo.scan_continuous{
  intensity = true,
  number = 3,
  -- str = "testing" -- seems not to like this?
})

local has_skt, skt = pcall(require, 'skt')
if not has_skt then
  print("No skt module for grabbing socket")
  os.exit()
end

-- Get ready to talk with the sensor
local transport = assert(skt.open{
  address = DEFAULT_IP,
  port = hokuyo.DEFAULT_PORT,
  tcp = true
})
local coro = coroutine.create(hokuyo.update)

-- Send 'em out
transport:send(pkt_version)
transport:send(pkt_parameters)
transport:send(pkt_status)
transport:send(pkt_off)
transport:send(pkt_status)
skt.poll({}, 200)
transport:send(pkt_on)
transport:send(pkt_status)
transport:send(pkt_scan_single)
-- Poll
while skt.poll({transport.fd}, 120) > 0 do
  -- io.stderr:write("* ACQUIRING... ")
  local data = transport:recv()
  -- io.stderr:write("RECEIVED ", data and #data or -1, '\n')
  -- io.stderr:write(data)
  -- io.stderr:write("\n**\n")
  local status, obj = coroutine.resume(coro, data)
  while type(obj)=='table' do
    io.stderr:write("\n!! ", tostring(obj.cmd), '\n')
    for k, v in pairs(obj) do
      if k~='distances' then
        print(k, type(v), v)
      else
        print("distances: ", v and #v)
      end
    end
    io.stderr:write"\n"
    status, obj = coroutine.resume(coro)
  end
  if not status then
    io.stderr:write("!! Coroutine: ", tostring(status), "[", obj, "]\n")
  end
end

print("\n== Stream On ==")
for state in hokuyo.command_it(transport, "stream_on") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Scan ==")
local ret = transport:send(pkt_continuous)
-- Poll
while skt.poll({transport.fd}, 120) > 0 do
  -- io.stderr:write("* ACQUIRING... ")
  local data = transport:recv()
  -- io.stderr:write("RECEIVED ", data and #data or -1, '\n')
  -- io.stderr:write(data)
  -- io.stderr:write("\n**\n")
  local status, obj = coroutine.resume(coro, data)
  while type(obj)=='table' do
    io.stderr:write("\n!! ", tostring(obj.cmd), '\n')
    for k, v in pairs(obj) do
      if k=='distances' then
        print("Distance: ", v and #v, unpack(v, 1, 4))
      elseif k=='intensities' then
        print("Intensity: ", v and #v, unpack(v, 1, 4))
      else
        print(k, type(v), v)
      end
    end
    io.stderr:write"\n"
    status, obj = coroutine.resume(coro)
  end
  if not status then
    io.stderr:write("!! Coroutine: ", tostring(status), "[", obj, "]\n")
  end
end


print("\n== version ==")
for state in hokuyo.command_it(transport, "version") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== parameters ==")
for state in hokuyo.command_it(transport, "parameters") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Stream On ==")
for state in hokuyo.command_it(transport, "stream_on") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== State ==")
for state in hokuyo.command_it(transport, "status") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== Single Scan ==")
for single in hokuyo.command_it(transport, "scan_single", false) do
  print("Got a single scan")
  for k, v in pairs(single) do
    if k~='distances' then
      print(k, v)
    else
      print("distances: ", v and #v)
    end
  end
end

print("\n== Stream Off ==")
for state in hokuyo.command_it(transport, "stream_off") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

print("\n== State ==")
for state in hokuyo.command_it(transport, "status") do
  for k, v in pairs(state) do
    print(k, v)
  end
end

transport:close()
