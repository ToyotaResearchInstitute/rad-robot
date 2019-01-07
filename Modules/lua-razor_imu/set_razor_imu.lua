#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack
local unix=require'unix'
local razor = require'razor_imu'
local settings = razor.settings

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

local function get_line(f)
  local line
  while not line do
    line = f:read"*line"
  end
  return line
end

print("First read", get_line(f_imu))

-- Adjust the rate
print("Adjusting rate")
local rate = 0
while rate~=100 do
  f_imu:write"r"
  repeat
    local line = get_line(f_imu)
    rate = tonumber(line:match"(%d+) Hz")
  until rate
end
print("Rate:", rate)

local values
repeat
  local line = get_line(f_imu)
  values = razor.line_values(line)
until #values > 0

-- local values = razor.line_values(line)
local n0 = #values
print("Original", n0)
print(unpack(values))
for _, s in ipairs(razor.all_settings) do
  print("Testing", s)
  -- Toggle this value
  f_imu:write(s)
  -- Wait for the toggle to take effect
  local n = n0
  while n==n0 do
    local line = get_line(f_imu)
    values = razor.line_values(line)
    n = #values
  end
  -- If this made the number of values go _down_,
  -- then we _disabled_ it, so retoggle if we want it _enabled_
  -- If this made the number of values go up,
  -- then we _enabled_ it, so retoggle if we want it _disabled_
  if (n < n0 and settings[s]) or (n > n0 and not settings[s]) then
    f_imu:write(s)
    while n~=n0 do
      local line = get_line(f_imu)
      values = razor.line_values(line)
      n = #values
    end
  end
  n0 = n
end

-- Ask for processed, not raw, values
if values[2]%1==0 and values[2]%1==0 then
  f_imu:write"c"
end

print()
print("Final", #values)
local line = get_line(f_imu)
-- Have the IMU stop writing... why???
-- f_imu:write" \n"
f_imu:close()
print(line)
print()
print("Values", unpack(razor.line_values(line)))
print()
local obj = assert(razor.parse(line))
for k, v in pairs(obj) do
  if type(v)=='table' then
    print(k, unpack(v))
  else
    print(k, v)
  end
end

