#!/usr/bin/env luajit
local nmea = require'nmea'
local dev = arg[1] or "/dev/ttyACM0"
for line in io.lines(dev) do
  print("NMEA sentence:", line)
  local obj = nmea.parse_sentence(line)
  local i = 0
  if type(obj)=='table' then
    for k,v in pairs(obj) do print(k, v) end
  end
end
