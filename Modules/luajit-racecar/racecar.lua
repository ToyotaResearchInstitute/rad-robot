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
local ROBOT_HOME = os.getenv"ROBOT_HOME" or '.'
local lib = {
  HOSTNAME = HOSTNAME,
  ROBOT_HOME = ROBOT_HOME
}

local mcl_obj = false
local lcm_obj = false
local function init(options)
  if not has_lcm then return false, "No MCL" end
  if type(options)~='table' then options = {} end
  -- MCL: localhost with ttl of 0, LCM: subnet with ttl of 1
  local MCL_ADDRESS, MCL_PORT = "239.255.65.56", 6556
  --
  mcl_obj = assert(lcm.init{
    _LCM_ADDRESS=MCL_ADDRESS,
    _LCM_PORT=MCL_PORT,
    -- TTL does not broadcast, unless we are houston
    ttl = IS_MAIN and 1 or 0,
    mtu = 'localhost'
  })
  if options.enable_lcm then
    local LCM_ADDRESS, LCM_PORT = "239.255.76.67", 7667
    lcm_obj = assert(lcm.init{
      _LCM_ADDRESS=LCM_ADDRESS,
      _LCM_PORT=LCM_PORT
    })
  end
  return true
end
lib.init = init

-- local exit_handler = false
-- lib.running = true
-- if has_signal then
--   local function shutdown()
--     if lib.running == false then
--       lib.running = nil
--       io.stderr:write"!! Double shutdown\n"
--       os.exit(type(exit_handler)=='function' and exit_handler() or 1)
--     elseif lib.running == nil then
--       io.stderr:write"!! Final shutdown\n"
--       os.exit(1)
--     end
--     lib.running = false
--   end
--   signal.signal("SIGINT", shutdown);
--   signal.signal("SIGTERM", shutdown);
-- else
--   io.stderr:write"No signal Support\n"
-- end

-- function lib.handle_shutdown(fn)
--   exit_handler = fn
-- end

local function announce(channel, str, cnt)
  if not mcl_obj then
    return false, "No socket"
  elseif type(channel)~='string' then
    return false, "No channel"
  elseif type(str)=='table' then
    str = has_logger and logger.encode(str)
  end
  -- Ensure that we have a string to send
  if type(str)~='string' then return false, "Bad serialize" end
  -- Use LCM to send
  local ret, err = mcl_obj:send(channel, str, tonumber(cnt))
  if not ret then return false, err end
  return #str
end
lib.announce = announce

function lib.log_announce(log, obj, channel)
  if not log then return announce(channel, obj) end
  if not channel then channel = log.channel end
  local str, cnt = log:write(obj, channel)
  local ret, err = announce(channel, str or obj, cnt)
  if not str then
    return false, cnt
  elseif not ret then
    return false, err
  end
  return ret
end

-- Automatic device finding of the IMU, vesc and joystick
local function populate_dev()
  local devices = {}
  for devname in io.popen"ls -1 /dev":lines() do
    local fullname = sformat("/dev/%s", devname)
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
        -- Flag arguments have a value
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
        -- Simply a positional argument
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
local function listen(options)
  if type(options)~='table' then options = {} end
  -- Add LCM channels to poll
  if type(options.channel_callbacks)=='table' then
    assert(has_logger, logger)
    for ch, cb in pairs(options.channel_callbacks) do
      print("Listening for", ch)
      assert(mcl_obj:cb_register(ch, cb, logger.decode))
    end
  end
  -- Add extra file descriptors to poll
  if type(options.fd_updates)=='table' then
    for fd, update in pairs(options.fd_updates) do
      assert(mcl_obj:fd_register(fd, update))
    end
  end
  local loop_rate = tonumber(options.loop_rate)
  local loop_rate1
  local fn_loop = type(options.fn_loop)=='function' and options.fn_loop
  local fn_debug = type(options.fn_debug)=='function' and options.fn_debug
  local t_update
  local dt_poll = 4 -- 250Hz
  local dt_loop
  local t_loop = 0
  local t_debug = 0
  local cnt_loop = 0
  local cnt_debug = 0
  -- debug_timeout measured in seconds
  local debug_timeout = tonumber(options.debug_timeout) or 1.0
  local status = true
  local err
  while lib.running do
    local t = time_us()
    if loop_rate then
      dt_loop = tonumber(t - t_loop) / 1e3
      loop_rate1 = max(1, min(loop_rate - dt_loop, dt_poll))
    else
      loop_rate1 = -1
    end
    status, err = mcl_obj:update(loop_rate1)
    t_update = time_us()
    if not status then lib.running = false; break end
    dt_loop = tonumber(t_update - t_loop)/1e3
    if fn_loop and dt_loop >= loop_rate then
      t_loop = t_update
      local ok, msg = fn_loop(t_loop, cnt_loop)
      -- if not ok then print(msg) end
      cnt_loop = cnt_loop + 1
    end
    local dt_debug = tonumber(t_update - t_debug)
    if dt_debug / 1e6 > debug_timeout then
      t_debug = t_update
      local jt = mcl_obj:jitter_info()
      if fn_debug then
        local msg_debug = fn_debug(t_debug, cnt_debug)
        cnt_debug = cnt_debug + 1
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
lib.listen = listen

local function replay(fnames, options)
  if type(options)~='table' then options = {} end
  local realtime = options.realtime
  local fn_loop = type(options.fn_loop)=='function' and options.fn_loop
  local fn_debug = type(options.fn_debug)=='function' and options.fn_debug
  local debug_timeout = tonumber(options.debug_timeout) or 1e3
  -- TODO: Add fn_loop, fn_debug
  local channel_callbacks = {}
  if type(options.channel_callbacks)=='table' then
    channel_callbacks = options.channel_callbacks
  end
  local co_play
  if type(fnames)=='string' then
    local log = logger.open(fnames)
    co_play = assert(log:play())
    -- co_play = assert(logger.play(fnames, options))
  elseif type(fnames)=='table' and #fnames>0 then
    co_play = assert(logger.play_many(fnames, options))
  else
    return false, "Invalid logs"
  end
  local t_debug = 0
  local cnt_debug = 0
  local t_log0, t_log1
  local t_host0
  while lib.running do
    if costatus(co_play)~='suspended' then break end
    local ok, str, ch, t_us = coresume(co_play)
    if not ok then
      io.stderr:write(sformat("Error: %s\n", str))
      break
    elseif not str then
      break
    end
    -- Run a callback
    local cb = channel_callbacks[ch]
    if type(cb)=='function' then
      local obj = assert(logger.decode(str))
      cb(obj, t_us)
    end
    if fn_loop then fn_loop(t_us) end
    local dt_debug = tonumber(t_us - t_debug)
    if dt_debug / 1e3 > debug_timeout then
      t_debug = t_us
      local jt = mcl_obj:jitter_info(true)
      if fn_debug then
        local msg_debug = fn_debug(t_debug, cnt_debug)
        cnt_debug = cnt_debug + 1
        if type(msg_debug)=='string' then
          tinsert(jt, msg_debug)
        end
      end
      if #jt > 0 then
        io.write('\n', tconcat(jt, '\n'), '\n')
        io.flush()
      end
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
    if realtime then usleep(dt_host) end
    t_log1 = t_us
  end
  return true
end
lib.replay = replay

if IS_MAIN then
  -- Always initialize
  init()
  local flags = parse_arg(arg, false)
  if flags.replay then
    print("Replay", flags.replay, unpack(flags))
    return replay({flags.replay, unpack(flags)},
      {
        realtime = flags.realtime,
      })
  elseif flags.spy then
    local function print_message(msg, ch, t_us)
      local tbl = {sformat("== [%s] @ %f", tostring(ch), tonumber(t_us) / 1.0e6)}
      for k, v in pairs(msg) do
        local str = sformat("[%s] = [%s]", tostring(k), tostring(v))
        table.insert(tbl, str)
      end
      print(table.concat(tbl, '\n'))
    end
    -- Print messages as they come
    if type(flags.spy)=='string' then
      listen{
        channel_callbacks = {
          [flags.spy] = print_message
        },
        debug_timeout = math.huge
      }
    elseif type(flags.spy)=='number' then
      listen{
        debug_timeout = flags.spy
      }
    end
  elseif flags[1] then
    -- Send houston message
    local msg = flags[1]
    local msg_houston = {}
    local ind_colon = msg:find":"
    if ind_colon then
      local submessages = {}
      for value in msg:gmatch"[^:]+" do
        table.insert(submessages, value)
      end
      msg_houston.id = submessages[1]
      msg_houston.evt = submessages[2]
      msg_houston.val = tonumber(submessages[3]) or submessages[3]
    else
      -- Simply send a message
      msg_houston.id = ''
      msg_houston.evt = msg
    end
    return announce('houston', msg_houston)
  end
end

return lib
