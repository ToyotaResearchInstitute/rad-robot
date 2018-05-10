local coresume = require'coroutine'.resume
local costatus = require'coroutine'.status
local min = require'math'.min
local max = require'math'.max
local schar = require'string'.char
local sformat = require'string'.format
local unpack = unpack or require'table'.unpack
local tconcat = require'table'.concat
local tinsert = require'table'.insert
local tremove = require'table'.remove
local has_logger, logger = pcall(require, 'logger')
local has_packet, packet = pcall(require, 'lcm_packet')
local fragment = has_packet and packet.fragment or function(ch, str)
  if #ch>255 then ch = ch:sub(1, 255) end
  return tconcat{
    schar(0x81), -- 1 element map
    schar(0xd9, #ch),
    ch, str
  }
end

local nan = 0/0
local RAD_TO_DEG = 180/math.pi
local DEG_TO_RAD = math.pi/180

local lib = {
  nan = nan,
  RAD_TO_DEG = RAD_TO_DEG,
  DEG_TO_RAD = DEG_TO_RAD
}

-- Jitter information
local jitter_counts, jitter_times = {}, {}

local MCL_ADDRESS, MCL_PORT = "239.255.65.56", 6556
-- local MCL_ADDRESS, MCL_PORT = "239.255.76.67", 7667
-- local unix = require'unix'
-- local time = require'unix'.time
local time_us = require'unix'.time_us
local usleep = require'unix'.usleep
local skt = require'skt'
local skt_mcl, err = skt.open{
  address = MCL_ADDRESS,
  port = MCL_PORT,
  -- ttl = 1
}
if not skt_mcl then
  io.stderr:write(string.format("MCL not available: %s\n", err))
end
local has_lcm, lcm = pcall(require, 'lcm')

local has_signal, signal = pcall(require, 'signal')
local exit_handler = false
lib.running = true
if has_signal then
  local function shutdown()
    if lib.running == false then
      lib.running = nil
      io.stderr:write"!! Double shutdown\n"
      os.exit(type(exit_handler)=='function' and exit_handler() or 1)
    elseif lib.running == nil then
      io.stderr:write"!! Final shutdown\n"
      os.exit(1)
    end
    lib.running = false
  end
  signal.signal("SIGINT", shutdown);
  signal.signal("SIGTERM", shutdown);
else
  io.stderr:write"No signal Support\n"
end

function lib.handle_shutdown(fn)
  exit_handler = fn
end

local function update_jitter(channel, t_us)
  local t_s = tonumber(t_us or time_us()) / 1e6
  local t_c = jitter_times[channel]
  if t_c then
    if #t_c >= 100 then tremove(t_c, 1) end
    tinsert(t_c, t_s)
    jitter_counts[channel] = jitter_counts[channel] + 1
  else
    jitter_times[channel] = {t_s}
    jitter_counts[channel] = 1
  end
end

local function announce(channel, str, cnt, t_us)
  if not (skt_mcl and channel) then
    return false, "No channel/socket"
  elseif type(str)=='table' then
    str = has_logger and logger.encode(str)
  end
  if type(str)~='string' then
    return false, "Bad serialize"
  end
  cnt = tonumber(cnt) or 0
  local msg = fragment(channel, str, cnt)
  local ret, err = skt_mcl:send_all(msg)
  update_jitter(channel, t_us)
  return #str
end
lib.announce = announce
function lib.log_announce(log, obj, channel)
  local cnt, t_us
  if log then
    channel = channel or log.channel
    obj, cnt, t_us = log:write(obj, channel)
  end
  return announce(channel, obj, cnt, t_us)
end

-- Calculate the jitter in milliseconds
local function get_jitter(ts)
  if #ts<2 then return nan, nan, nan end
  local diffs, adiff = {}, 0
  for i=2,#ts do
    local d = ts[i] - ts[i-1]
    adiff = adiff + d
    tinsert(diffs, d)
  end
  adiff = adiff / #diffs
  local jMin, jMax = min(unpack(diffs)), max(unpack(diffs))
  -- milliseconds
  return adiff*1e3, (jMin - adiff)*1e3, (jMax - adiff)*1e3
end
lib.get_jitter = get_jitter

local function jitter_tbl(info)
  info = info or {}
  for ch, ts in pairs(jitter_times) do
    local avg, jitterA, jitterB = get_jitter(ts)
    -- tinsert(info, string.format(
    --   "%s\t%3d\t%5.1f Hz\t%+6.2f ms\t%3d ms\t%+6.2f ms",
    --   ch, #ts, 1e3/avg, jitterA, avg, jitterB))
    tinsert(info, sformat(
      "%s\t%5.1f Hz\t%+6.2f ms\t%6.2f ms\t%+6.2f ms",
      ch, 1e3/avg, jitterA, avg, jitterB))
  end
  -- TODO: Remove stale entries
  return info
end
lib.jitter_tbl = jitter_tbl

-- Automatic device finding of the IMU, vesc and joystick
local function populate_dev()
  local devices = {}
  for devname in io.popen"ls -1 /dev":lines() do
    local fullname = string.format("/dev/%s", devname)
    local is_valid = false
    if devname=="imu" then
      devices.imu = fullname
    elseif devname=="vesc" then
      devices.vesc = fullname
    elseif devname=="ublox" then
      devices.ublox = fullname
    elseif devname:match"cu%.usbmodem%d+" then
      is_valid = true
    elseif devname:match"ttyACM%d+" then
      is_valid = true
    end
    if is_valid then table.insert(devices, fullname) end
  end
  devices.joystick = io.popen"ls -1 /dev/input/js* 2>/dev/null":read"*line"
  return devices
end

function lib.parse_arg(arg, use_dev)
  local flags = {
    home = os.getenv"RACECAR_HOME" or '.',
    debug = tonumber(os.getenv"DEBUG")
  }
  do
    local i = 1
    while i<=#arg do
      local a = arg[i]
      i = i + 1
      local flag = a:match"^--([%w_]+)"
      if flag then
        local val = arg[i]
        if not val then break end
        flags[flag] = tonumber(val) or val
      else
        tinsert(flags, a)
      end
    end
  end
  if use_dev~=false then
    local devices = populate_dev(flags)
    if type(devices) == 'table' then
      flags.imu = flags.imu or devices.imu
      flags.vesc = flags.vesc or devices.vesc
      flags.gps = flags.gps or devices.ublox
      flags.js = flags.js or devices.joystick
    end
  end
  return flags
end

-- Run through the log
--[[
function lib.play(it_log, realtime, cb)
  local t_log0, t_log1
  local t_host0
  local t_send = -math.huge
  local dt_send = 1e6 / 5
  for str, ch, t_us, count in it_log do
    if not lib.running then break end
    local t_host = time_us()
    if not t_log0 then
      t_log0 = t_log0 or t_us
      t_host0 = t_host
    end
    local dt_log = t_log1 and tonumber(t_us - t_log1) or 0
    local dt0_log = tonumber(t_us - t_log0)
    local dt0_host = tonumber(t_host - t_host0)
    local lag_to_host = dt0_log - dt0_host
    local dt_host = dt_log + lag_to_host
    local ret = realtime and usleep(dt_host)
    t_log1 = t_us
    local ret = realtime and cb and cb(dt0_log)
  end
end
--]]

-- Rate: loop rate in milliseconds
function lib.listen(cb_tbl, loop_rate, loop_fn)
  assert(has_lcm, lcm)
  assert(has_logger, logger)
  local lcm_obj = assert(lcm.init(MCL_ADDRESS, MCL_PORT))
  if type(cb_tbl)=='table' then
    for ch, fn in pairs(cb_tbl) do
      assert(lcm_obj:register(ch, fn, logger.decode))
    end
  end
  loop_rate = tonumber(loop_rate) or -1
  loop_fn = type(loop_fn)=='function' and loop_fn
  -- local t0, t1 = 0, 0
  local t_loop = 0
  local t_debug = 0
  local dt_debug = 1e6
  while lib.running do
    -- t0 = time_us()
    local loop_rate1 = max(0, loop_rate + tonumber(t_loop - time_us())/1e3)
    local status = lcm_obj:update(loop_rate1)
    if not status then
      lib.running = false
    end
    t_loop = time_us()
    if loop_rate then
      if loop_fn then loop_fn() end
      update_jitter("lcm_loop", t_loop)
    end
    if tonumber(t_loop - t_debug) > dt_debug then
      io.write(tconcat(jitter_tbl(), '\n'), '\n')
      t_debug = time_us()
    end
  end
end

function lib.play(fnames, realtime, update, cb)
  local co = assert(logger.play(fnames, false, update))
  local t_log0, t_log1
  local t_host0
  local t_send = -math.huge
  local dt_send = 1e6 / 5
  repeat
    local ok, str, ch, t_us, count = coresume(co)
    if not ok then
      io.stderr:write(string.format("Error: %s\n", str))
      break
    elseif costatus(co)~='suspended' then
      io.stderr:write"Dead coro\n"
      break
    end
    local t_host = time_us()
    if not t_log0 then
      t_log0 = t_log0 or t_us
      t_host0 = t_host
    end
    local dt_log = t_log1 and tonumber(t_us - t_log1) or 0
    local dt0_log = tonumber(t_us - t_log0)
    local dt0_host = tonumber(t_host - t_host0)
    local lag_to_host = dt0_log - dt0_host
    local dt_host = dt_log + lag_to_host
    local ret = realtime and usleep(dt_host)
    t_log1 = t_us
    local ret = realtime and cb and cb(dt0_log, ch)
  until not lib.running
end

return lib
