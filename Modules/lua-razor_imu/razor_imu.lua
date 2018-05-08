-- https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide

local lib = {}

-- Input payload table
local names = {
  "timeM",
  "accelX", "accelY", "accelZ",
  "gyroX", "gyroY", "gyroZ",
  "magX", "magY", "magZ",
--  "qw", "qx", "qy", "qz",
--  "pitch", "roll", "yaw",
--  "heading"
}
local nvals = #names

local function line2vals(line)
  local vals = {}
  for val in line:gmatch"[^%, ]+" do
    table.insert(vals, tonumber(val) or false)
  end
  return vals
end
lib.line_values = line2vals

local function parse(line)
  local vals = line2vals(line)
--[[
  if #vals ~= nvals then
    io.stderr:write(string.format(
      "Expected %d Got %d\n", nvals, #vals))
  end
--]]
  -- Check the length of vals
  local tbl = {}
  tbl.timeM = vals[1]
  local ival = 2
  if ival>#vals then return tbl end
  tbl.accel = {}
  for i=1,3 do
    tbl.accel[i] = vals[ival]
    ival = ival + 1
  end
  if ival>#vals then return tbl end
  tbl.gyro = {}
  for i=1,3 do
    tbl.gyro[i] = vals[ival]
    ival = ival + 1
  end
  if ival>#vals then return tbl end
  tbl.mag = {}
  for i=1,3 do
    tbl.mag[i] = vals[ival]
    ival = ival + 1
  end
  if ival>#vals then return tbl end
  tbl.q = {}
  for i=1,4 do
    tbl.q[i] = vals[ival]
    ival = ival + 1
  end
  if ival>#vals then return tbl end
  -- Comes in as pitch roll yaw
  tbl.rpy = {
    vals[ival+1],
    vals[ival],
    vals[ival+2]
  }
  ival = ival + 3
  if ival>#vals then return tbl end
  tbl.heading = vals[ival]
  return tbl
end
lib.parse = parse

function lib.update(new_data)
  local str = ''
  while true do
    if type(new_data)=='string' then
      str = str..new_data
    end
    local istart = str:find('\n', 1, true)
    local obj
    if istart then
      local line = str:sub(1, istart-1)
      str = str:sub(istart + 1)
      obj = parse(line)
    end
    new_data = coroutine.yield(obj)
  end
end

local function co_service(devname)
  local f = io.open(devname)
  coroutine.yield(f)
  for line in f:lines() do
    coroutine.yield(parse(line))
  end
end

function lib.service(devname, wrap)
  if type(devname) ~= 'string' then
    return false, "Bad device name"
  end
  if wrap then
    local fn =  coroutine.wrap(co_service)
    fn(devname)
    return fn
  else
    local coro = coroutine.create(co_service)
    coroutine.resume(coro, devname)
    return coro
  end
end

return lib
