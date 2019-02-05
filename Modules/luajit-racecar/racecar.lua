#!/usr/bin/env luajit
local IS_MAIN = arg[-1]=='luajit' and arg[0]:match"[^/]?racecar.lua$"=='racecar.lua' and ... ~= 'racecar'
-- print('IS_MAIN', IS_MAIN, arg[0])

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
local has_lcm, lcm = pcall(require, 'lcm')
local has_packet, lcm_packet = pcall(require, 'lcm_packet')
local has_signal, signal = pcall(require, 'signal')
local time_us = require'unix'.time_us
local usleep = require'unix'.usleep
-- Simple msgpack payload with no fragmentation
local function mp_fragment(ch, str, str_sz, cnt)
  -- Truncate the channel name (msgpack limited)
  local ch_sz = #ch
  if ch_sz>255 then ch = ch:sub(1, 255) end
  return tconcat{
    schar(0x81), -- 1 element map
    schar(0xd9, ch_sz),
    ch, str
  }
end

local HOSTNAME = io.popen"hostname":read"*line"

local nan = 0/0
local ROBOT_HOME = os.getenv"ROBOT_HOME" or '.'
local lib = {
  nan = nan,
  RPM_PER_MPS = 5220,
  HOSTNAME = HOSTNAME,
  ROBOT_HOME = ROBOT_HOME
}

-- Jitter information
local jitter_counts, jitter_times = {}, {}

local skt_mcl, skt_lcm, msg_partitioner
local function init(options)
  if type(options)~='table' then options = {} end
  -- MCL: localhost with ttl of 0, LCM: subnet with ttl of 1
  if not has_lcm then return false, "No MCL" end
  local err
  local has_skt, skt = pcall(require, 'skt')
  if has_skt then
    local MCL_ADDRESS, MCL_PORT = "239.255.65.56", 6556
    skt_mcl, err = skt.open{
      address = MCL_ADDRESS,
      port = MCL_PORT,
      ttl = IS_MAIN and 1 or 0
    }
    if skt_mcl then
      local mtu = options.mtu or 'localhost'
      msg_partitioner = lcm_packet.new_partitioning(mtu)
    else
      io.stderr:write(string.format("MCL not available: %s\n",
                                    tostring(err)))
    end
    local LCM_ADDRESS, LCM_PORT = "239.255.76.67", 7667
    skt_lcm, err = skt.open{
      address = LCM_ADDRESS,
      port = LCM_PORT,
      -- ttl = 1
    }
    if not skt_lcm then
      io.stderr:write(string.format("LCM not available: %s\n",
                                    tostring(err)))
    end
  end
  return true
end
lib.init = init

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
  if not skt_mcl then
    return false, "No socket"
  elseif type(channel)~='string' then
    return false, "No channel"
  elseif type(str)=='table' then
    str = has_logger and logger.encode(str)
  end
  if type(str)~='string' then
    return false, "Bad serialize"
  end
  cnt = tonumber(cnt) or jitter_counts[channel] or 0
  local msg = msg_partitioner:fragment(channel, str, #str, cnt)
  local ret, err = skt_mcl:send_all(msg)
  if not ret then return false, err end
  update_jitter(channel, t_us)
  return #str
end
lib.announce = announce

function lib.log_announce(log, obj, channel)
  local str, cnt, t_us
  if log then
    if not channel then channel = log.channel end
    str, cnt, t_us = log:write(obj, channel)
  end
  local ret, err = announce(channel, str or obj, cnt, t_us)
  if log and not str then
    -- Logging error
    return false, cnt
  elseif not ret then
    -- Sending error
    return false, err
  else
    -- OK
    return true
  end
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

local function parse_arg(arguments, use_dev)
  if type(arguments)~='table' then
    return false, "Need argument as table"
  end
  local flags = {
    debug = tonumber(os.getenv"DEBUG")
  }
  do
    local i = 1
    local n_args = #arguments
    while i <= n_args do
      local a = arguments[i]
      local flag = a:match"^%-%-([%w_]+)"
      if flag then
        local val = arguments[i + 1]
        if not val then break end
        if val:lower() == 'true' then
          flags[flag] = true
        elseif val:lower() == 'false' then
          flags[flag] = false
        else
          flags[flag] = tonumber(val) or val
        end
        i = i + 2
      else
        tinsert(flags, a)
        i = i + 1
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
lib.parse_arg = parse_arg

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
function lib.listen(options)
  assert(has_lcm, lcm)
  local lcm_obj = assert(lcm.init{skt = skt_mcl})
  if type(options)~='table' then options = {} end
  -- Add LCM channels to poll
  if type(options.channel_callbacks)=='table' then
    assert(has_logger, logger)
    for ch, cb in pairs(options.channel_callbacks) do
      print("Listening for", ch)
      assert(lcm_obj:cb_register(ch, cb, logger.decode))
    end
  end
  -- Add extra file descriptors to poll
  if type(options.fd_updates)=='table' then
    for fd, update in pairs(options.fd_updates) do
      assert(lcm_obj:fd_register(fd, update))
    end
  end
  local loop_rate = tonumber(options.loop_rate)
  local loop_rate1
  local fn_loop = type(options.fn_loop)=='function' and options.fn_loop
  local fn_debug = type(options.fn_debug)=='function' and options.fn_debug
  local t_update
  local dt_poll = 4 -- 250Hz
  local t_fn = 0
  local dt_fn
  local t_debug = 0
  local debug_rate = tonumber(options.debug_rate) or 1e3
  local status = true
  local err
  while lib.running do
    local t = time_us()
    if loop_rate then
      dt_fn = tonumber(t - t_fn) / 1e3
      loop_rate1 = max(1, min(loop_rate - dt_fn, dt_poll))
    else
      loop_rate1 = -1
    end
    status, err = lcm_obj:update(loop_rate1)
    t_update = time_us()
    if not status then lib.running = false; break end
    dt_fn = tonumber(t_update - t_fn)/1e3
    if fn_loop and dt_fn >= loop_rate then
      t_fn = t_update
      -- update_jitter("lcm_loop", t_fn)
      fn_loop(t_fn)
    end
    local dt_debug = tonumber(t_update - t_debug)
    if dt_debug / 1e3 > debug_rate then
      t_debug = t_update
      local jt = jitter_tbl()
      if fn_debug then
        local msg_debug = fn_debug(t_debug)
        if type(msg_debug)=='string' then
          tinsert(jt, msg_debug)
        end
      end
      if #jt > 0 then
        io.write('\n', tconcat(jt, '\n'), '\n')
        io.flush()
      end
    end
  end
  return status, err
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

if IS_MAIN then
  init()
  local flags = parse_arg(arg, false)
  local msg = flags[1]
  if msg then
    print("Sending", msg)
    assert(announce('houston', {evt=msg}))
  end
end

return lib
