#!/usr/bin/env luajit
local devices = {}
local is_macos = false
for devname in io.popen("ls -1 /dev"):lines() do
  if devname:match"cu%.usbmodem%d+" then
    is_macos = true
    table.insert(devices, string.format("/dev/%s", devname))
  elseif devname:match"ttyACM%d+" then
    table.insert(devices, string.format("/dev/%s", devname))
  end
end

local ttyname = assert(arg[1] or devices[1], "Need a device")
local razor_imu = require'razor_imu'

local coro = razor_imu.service(ttyname, true)

local t0 = os.time()
local dt = 0
local n = 0
for measurement in coro do
  io.write"\n"
  if type(measurement)=='table' then
    for k,v in pairs(measurement) do
      if type(v)=='table' then print(k, unpack(v)) else print(k, v) end
    end
    n = n + 1
  else
    io.stderr:write("Error: ", measurement, "\n")
  end
  local t = os.time()
  dt = os.difftime(t, t0)
  if dt >= 30 then break end
end
print("Sample rate", n/dt)
