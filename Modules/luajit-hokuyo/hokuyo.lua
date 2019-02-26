local lib = {}

lib.DEFAULT_IP = "192.168.0.10"
lib.DEFAULT_PORT = 10940

local has_bit, bit = pcall(require, 'bit')
local lshift = has_bit and bit.lshift
local tinsert = require'table'.insert
local cos = require'math'.cos
local sin = require'math'.sin
local floor = require'math'.floor

local command_names = {
  VV = 'version',
  PP = 'parameters',
  II = 'status',
  BM = 'stream_on',
  QT = 'stream_off',
  GD = 'scan_single',
  GE = 'scan_single',
  MD = 'scan_continuous',
  ME = 'scan_continuous',
}
lib.command_names = command_names

local thetas = {}
local np = 1081 -- NOTE: Be careful about other hokuyo models
-- for i=-floor(np/2), floor(np/2) do
for i=floor(np/2), -floor(np/2),-1 do
  tinsert(thetas, math.rad(0.25 * i))
end
assert(#thetas==np, "Bad angle calculation")

-- distances are in millimeters
-- Planar, so no zs component
local function distances2points(distances)
  local xs, ys, zs, hits = {}, {}, {}, {}
  local MAX_DIST = 10
  for i, d0 in ipairs(distances) do
    local th = thetas[i]
    local c, s = cos(th), sin(th)
    local hit = d0 ~= 0xFFFD
    local d = hit and (d0/1e3) or MAX_DIST
    --
    tinsert(hits, hit)
    tinsert(xs, d * c)
    tinsert(ys, d * s)
    tinsert(zs, 0)
  end
  return xs, ys, zs, hits
end
lib.distances2points = distances2points

local function simple_packet(id, str)
  local tbl = {
    id,
    type(str)=='string' and str or '',
    '\n'
  }
  local cmd = table.concat(tbl)
  if #cmd > 64 then
    return false, "Long packet"
  end
  return cmd
end

for code,cmd in pairs(command_names) do
  lib[cmd] = function(str)
    return simple_packet(code, str)
  end
end

function lib.scan_single(intensity, start, stop, str, cluster)
  local tbl = {
    intensity and "GE" or "GD",
    string.format("%04d", start or 0),
    string.format("%04d", stop or 1080),
    string.format("%02d", cluster or 0),
    type(str)=='string' and str or '',
    '\n'
  }
  if #tbl[2]~=4 or #tbl[3]~=4 or #tbl[4]~=2 then
    return false, "Bad packet"
  end
  local cmd = table.concat(tbl)
  if #cmd > 64 then
    return false, "Long packet"
  end
  return cmd
end

function lib.scan_continuous(settings)
  local tbl = {
    settings.intensity and "ME" or "MD",
    string.format("%04d", settings.start or 0),
    string.format("%04d", settings.stop or 1080),
    string.format("%02d", settings.cluster or 0),
    string.format("%01d", settings.interval or 0),
    string.format("%02d", settings.number or 0),
    type(settings.str)=='string' and settings.str or '',
    '\n'
  }
  if #tbl[2]~=4 or #tbl[3]~=4
    or #tbl[4]~=2 or #tbl[5]~=1
    or #tbl[6]~=2 then
    return false, "Bad packet"
  end
  local cmd = table.concat(tbl)
  if #cmd > 64 then
    return false, "Long packet"
  end
  return cmd
end

local function line2time(line)
  -- Convert to timestamp in milliseconds
  local sum = 0
  -- Timestamp is always four bytes
  local n = 4
  for i=1,n do
    sum = sum + lshift(line:byte(i) - 0x30, 6 * (n - i))
  end
  return sum
end

local function decode3byte(a, b, c)
  return lshift(a - 0x30, 12) + lshift(b - 0x30, 6) + (c - 0x30)
end

local function scan2returns(scan)
  if type(scan)~='table' or #scan==0 then return end
  scan = table.concat(scan)
  local nscan = #scan
  -- Distance or distance+intensities
  local distances, intensities
  if nscan == 3243 then
    -- Convert to millimeters
    distances = {}
    -- Three byte encoding
    for i=1, nscan, 3 do
      table.insert(distances, decode3byte(scan:byte(i, i+2)))
    end
  else
    distances, intensities = {}, {}
    for i=1, nscan, 6 do
      table.insert(distances, decode3byte(scan:byte(i, i+2)))
      table.insert(intensities, decode3byte(scan:byte(i+3, i+5)))
    end
  end
  return distances, intensities
end

-- Enumerate the parts of the packet
local PKT_START = 0
local PKT_MSG = 1
local PKT_STATUS = 2
local PKT_CHECKSUM = 3
local PKT_SCAN = 4
local PKT_TIME = 5
local PKT_MAP = 6
local PKT_END = 7
local PKT_CMD_MULTISCAN = 8
local PKT_CMD_SINGLESCAN = 9
local function update(str, obj, pkt)

end

function lib.update(new_data)
  local str = ''
  local pkt_state = PKT_START
  local pkt_done = false
  local pkt_payload
  -- Keep the coroutine going forever
  while true do
    -- Add any new data
    if type(new_data) == 'string' then
      str = str..new_data
      new_data = false
    end
    -- io.stderr:write("STR: ", str, '\n')
    -- Line by line for hokuyo
    local iline = str:find('\n', 1, true)
    local line
    if iline then
      -- io.stderr:write("ILINE: ", iline, '\n')
      -- Process this line
      line = str:sub(1, iline - 1)
      -- Prep for next time
      str = str:sub(iline + 1)
      -- Check packet end
      if #line == 0 then pkt_state = PKT_END end
    end
    while line do
      if pkt_state == PKT_START then
        -- io.stderr:write("PKT_START: ", line, '\n')
        pkt_payload = {}
        local command = line:sub(1, 2)
        pkt_payload.cmd = command
        -- pkt_payload.cmd = command_names[command]
        line = line:sub(3)
        -- Check the next state
        local cmd1 = command:sub(1, 1)
        if cmd1 == 'G' then
          pkt_state = PKT_CMD_SINGLESCAN
        elseif cmd1 == 'M' then
          pkt_state = PKT_CMD_MULTISCAN
        elseif #line > 0 then
          pkt_state = PKT_MSG
        else
          pkt_state = PKT_STATUS
        end
      elseif pkt_state == PKT_CMD_SINGLESCAN then
        -- io.stderr:write("PKT_CMD_SINGLESCAN: ", line, '\n')
        pkt_payload.scan = {}
        line = line:sub(12)
        pkt_state = #line > 0 and PKT_MSG or PKT_STATUS
      elseif pkt_state == PKT_CMD_MULTISCAN then
        -- io.stderr:write("PKT_CMD_MULTISCAN: ", line, '\n')
        pkt_payload.scan = {}
        -- pkt_payload.remaining = tonumber(line:sub(12, 13)) or false
        -- line = line:sub(14)
        -- pkt_state = #line > 0 and PKT_MSG or PKT_STATUS
        pkt_payload.remaining = tonumber(line:sub(12)) or false
        line = false
        pkt_state = PKT_STATUS
      elseif pkt_state == PKT_MSG then
        -- io.stderr:write("PKT_MSG: ", line, '\n')
        pkt_payload.message = line
        line = false
        pkt_state = PKT_STATUS
      elseif pkt_state == PKT_STATUS then
        -- io.stderr:write("PKT_STATUS: ", line, '\n')
        pkt_payload.status = line:sub(1, 2)
        line = false
        pkt_state = pkt_payload.scan and PKT_TIME or PKT_MAP
      elseif pkt_state == PKT_TIME then
        -- io.stderr:write("PKT_TIME: ", line, '\n')
        pkt_payload.TIME = line2time(line)
        line = false
        pkt_state = PKT_SCAN
      elseif pkt_state == PKT_SCAN then
        -- io.stderr:write("PKT_SCAN: ", line, '\n')
        table.insert(pkt_payload.scan, line:sub(1, -2))
        line = false
      elseif pkt_state == PKT_MAP then
        -- io.stderr:write("PKT_MAP: ", line, '\n')
        -- Add the key
        local key, val, _ = line:match"(%w+)%:(.+)%;(.)"
        if not key then
          io.stderr:write"No key/value\n"
        elseif key=="TIME" then
          pkt_payload[key] = line2time(val)
        else
          pkt_payload[key] = tonumber(val) or val
        end
        line = false
      elseif pkt_state == PKT_END then
        -- io.stderr:write("PKT_END: ", line, '\n')
        pkt_state = PKT_START
        line = false
        pkt_payload.distances, pkt_payload.intensities = scan2returns(pkt_payload.scan)
        pkt_payload.scan = nil
        pkt_done = true
        break
      end
      line = line and (#line > 0) and line
    end
    if pkt_done then
      new_data = coroutine.yield(pkt_payload)
      pkt_done = false
    elseif not iline then
      new_data = coroutine.yield()
    end
  end
  return false
end

local has_skt, skt = pcall(require, 'skt')
if has_skt then

  local function co_command(transport, cmd, arg)
    local f = lib[cmd]
    if type(f) ~= 'function' then
      return false, "Command not supported"
    end
    local pkt, err = f(arg)
    if not pkt then return false, err end
    -- init
    local coro = coroutine.create(lib.update)

    coroutine.yield()

    local ret = transport:send(pkt)
    -- Poll for data
    while skt.poll({transport.fd}, 25) > 0 do
      -- print("\n* DATA *")
      -- io.write(response)
      -- print("Received", response and #response)
      local data = transport:recv()
      local status, obj = coroutine.resume(coro, data)
      while type(obj)=='table' do
        coroutine.yield(obj)
        if command_names[obj.cmd]==cmd then
          break
        end
        status, obj = coroutine.resume(coro)
      end
    end
  end

  function lib.command_it(transport, cmd, arg)
    local co_single = coroutine.wrap(co_command)
    -- Set the command
    co_single(transport, cmd, arg)
    return co_single
  end

end

return lib