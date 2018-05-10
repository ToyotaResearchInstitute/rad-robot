#!/usr/bin/env luajit

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local jitter_tbl = racecar.jitter_tbl
local log_announce = racecar.log_announce

local time = require'unix'.time
local poll = require'unix'.poll
local logger = require'logger'
local skt = require'skt'
local hokuyo = require'hokuyo'

local channel = 'hokuyo'
local log_dir = (os.getenv"RACECAR_HOME" or '.').."/logs"
local log = flags.log~=0 and assert(logger.new(channel, log_dir))

local skt_hokuyo = assert(skt.open{
  address = hokuyo.DEFAULT_IP,
  port = hokuyo.DEFAULT_PORT,
  tcp = true
})
local fds = {skt_hokuyo.fd}

local function entry()
  -- Grab the parameters to save
  local hokuyo_params = {}
  for state in hokuyo.command_it(skt_hokuyo, "parameters") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  for state in hokuyo.command_it(skt_hokuyo, "version") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  for state in hokuyo.command_it(skt_hokuyo, "stream_on") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  for state in hokuyo.command_it(skt_hokuyo, "status") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  for k, v in pairs(hokuyo_params) do print(k, v) end
  assert(log_announce(log, hokuyo_params, 'hokuyo_info'))
  local pkt_continuous = hokuyo.scan_continuous{
    intensity = true,
  }
  -- Turn on the Hokuyo and ask for continuous scans
  local ret = skt_hokuyo:send(pkt_continuous)
end

local coro = coroutine.create(hokuyo.update)
local function process(data, t)
  repeat
    local status, obj = coroutine.resume(coro, data)
    data = nil
    if not status then
      io.stderr:write(obj, '\n')
      return status, obj
    elseif type(obj) ~= 'table' then
      -- io.stderr:write('bad hokuyo obj\n')
    elseif obj.status=='00' then
      log_announce(log, obj, 'hokuyo_info')
    else
      log_announce(log, obj, channel)
    end
  until not obj
end

local function exit()
  local hokuyo_params = {}
  for state in hokuyo.command_it(skt_hokuyo, "stream_off") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  for state in hokuyo.command_it(skt_hokuyo, "status") do
    for k, v in pairs(state) do
      if k:upper()==k then hokuyo_params[k] = v end
    end
  end
  log_announce(log, hokuyo_params, 'hokuyo_info')

  if log then log:close() end
  skt_hokuyo:close()
end
racecar.handle_shutdown(exit)

entry()
local t_debug = time()
local n = 0
while racecar.running do
  local rc, ready = poll(fds, 1e3)
  local t_poll = time()
  if rc and ready then
    local data = skt_hokuyo:recv()
    process(data, t_poll)
  elseif rc then
    io.stderr:write("No data!\n")
  else
    io.stderr:write("uh oh\n")
  end
  local dt_debug = t_poll - t_debug
  if dt_debug > 1 then
    t_debug = t_poll
    local info = jitter_tbl()
    io.write(table.concat(info, '\n'), '\n')
  end
end
exit()
