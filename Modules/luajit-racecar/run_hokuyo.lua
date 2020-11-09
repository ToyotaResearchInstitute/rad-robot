#!/usr/bin/env luajit

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
racecar.init()
local log_announce = racecar.log_announce

local time = require'unix'.time
local logger = require'logger'
local skt = require'skt'
local hokuyo = require'hokuyo'

local channel = 'hokuyo'
local log = flags.log~=0 and assert(logger.new(channel, racecar.ROBOT_HOME.."/logs"))

local skt_hokuyo = assert(skt.open{
  address = hokuyo.DEFAULT_IP,
  port = hokuyo.DEFAULT_PORT,
  tcp = true
})

-- Should be part of a Hokuyo obj that has a __gc exit method
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

-- Should be a __gc function
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
-- racecar.handle_shutdown(exit)

local coro = coroutine.create(hokuyo.update)
local function process(data)
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
  return true
end

local function on_hokuyo(e)
  if e~=1 and skt_hokuyo then
    print("Reading", e)
    exit()
    return os.exit(false, true)
  end
  local pkt, status = skt_hokuyo:recv()
  if not pkt then
    io.stderr:write(status, '\n')
    return
  end
  process(pkt, time())
end

local fd_updates = {
  [skt_hokuyo.fd] = on_hokuyo
}

entry()
racecar.listen{
  fd_updates = fd_updates,
}
exit()
