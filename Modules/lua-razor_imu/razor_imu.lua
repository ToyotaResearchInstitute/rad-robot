-- https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide

local lib = {}
local tinsert = require'table'.insert

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
lib.names = names
local nvals = #names

local settings = {}
for _, name in ipairs(names) do
  if name:match"^accel" then
    settings['a'] = true
  elseif name:match"^gyro" then
    settings['g'] = true
  elseif name:match"^mag" then
    settings['m'] = true
  elseif name:match"^q[wxyz]$" then
    settings['q'] = true
  elseif name=='pitch' or name=='roll' or name=='yaw' then
    settings['e'] = true
  elseif name=='heading' then
    settings['h'] = true
  end
end

lib.settings = settings
lib.all_settings = {'a', 'g', 'm', 'q', 'e', 'h'}

local function line2vals(line)
  local vals = {}
  for val in line:gmatch"[^%, ]+" do
    tinsert(vals, tonumber(val) or false)
  end
  return vals
end
lib.line_values = line2vals

local function parse(line, tbl)
  local vals = line2vals(line)
  if not tbl then tbl = {} end
  if #vals ~= nvals then
    return false, string.format(
      "Expected %d Got %d\n", nvals, #vals)
  end
  -- Check the length of vals
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

-- pkt: new data
-- obj: table in which to store info
-- buffer: current data
local function update(pkt, obj, buffer)
  -- Append new data to our buffer
  if type(buffer)~='string' then buffer = '' end
  if type(pkt)=='string' then buffer = buffer..pkt end
  local iline = buffer:find('\n', 1, true)
  -- If no complete packet exists, just return
  if not iline then return obj, buffer end
  -- Save the buffer
  local buffer1 = buffer:sub(iline + 1)
  -- Take the line
  local line = buffer:sub(1, iline-1)
  -- Return the data and the buffer
  obj = parse(line, obj)
  return obj, buffer1
end
lib.update = update

local coyield = require'coroutine'.yield
function lib.co_update(str)
  local buffer = ''
  local obj = {}
  while true do
    obj, buffer = update(str, obj, buffer)
    str = coyield(obj)
  end
end

local function co_service(devname)
  local f = io.open(devname)
  coyield(f)
  for line in f:lines() do
    coyield(parse(line))
  end
  f:close()
end

function lib.service(devname, wrap)
  if type(devname) ~= 'string' then
    return false, "Bad device name"
  end
  if wrap then
    local fn = coroutine.wrap(co_service)
    fn(devname)
    return fn
  else
    local coro = coroutine.create(co_service)
    coroutine.resume(coro, devname)
    return coro
  end
end

return lib
