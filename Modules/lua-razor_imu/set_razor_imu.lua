#!/usr/bin/env luajit
local unix=require'unix'
local razor = require'razor_imu'
local settings = {'a', 'g', 'm', 'q', 'e'}
local fname = arg[1] or "/dev/imu"
print("Opening imu...")
local f_imu = assert(io.open(fname, "w+"))
print("Opened IMU", f_imu)
f_imu:setvbuf"no"
f_imu:flush()
local fd_imu = unix.fileno(f_imu)
print("Polling IMU, fd:", fd_imu)
local ret = assert(unix.poll({fd_imu}, 2e3))
print("ret", ret)
if ret==0 then
  print("Asking IMU to publish")
  -- IMU is not publishing, so we must enable this
  f_imu:write" "
  print("Waiting for a response")
  assert(unix.poll({fd_imu}, 2e3))
end

local line = f_imu:read"*line"
line = f_imu:read"*line"
print(line)

local function get_line(f)
local line
repeat
line = f:read"*line"
until line
return line
end

-- Adjust the rate
print("Adjusting rate")
local rate = 0
while rate~=100 do
  f_imu:write"r"
  repeat
    line = get_line(f_imu)
    rate = tonumber(line:match"(%d+) Hz")
  until rate
end
print("Rate:", rate)

local values
repeat
line = get_line(f_imu)
values = razor.line_values(line)
until #values>0

local values = razor.line_values(line)
local n0 = #values
print("Original", n0)
print(unpack(values))
for _, s in ipairs(settings) do
  print("Testing", s)
  -- Flush the buffer, first
  f_imu:write(s)
  local n
  repeat
    local line = get_line(f_imu)
    values = razor.line_values(line)
    n = #values
  until n~=n0
  if n < n0 then
    f_imu:write(s)
    repeat
      local line = get_line(f_imu)
      values = razor.line_values(line)
      n = #values
    until n==n0
  end
  n0 = n
end

-- Ask for processed, not raw, values
if values[2]%1==0 and values[2]%1==0 then
  f_imu:write"c"
end

print()
print("Final", #values)
line = f_imu:read"*line"
print(line)
print()
print(unpack(razor.line_values(line)))
print()
for k, v in pairs(razor.parse(line)) do
  if type(v)=='table' then
    print(k, unpack(v))
  else
    print(k, v)
  end
end
f_imu:write" \n"
f_imu:close()
