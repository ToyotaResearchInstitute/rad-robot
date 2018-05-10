#!/usr/bin/env luajit
local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local jitter_tbl = racecar.jitter_tbl
local log_announce = racecar.log_announce

local vicon = require'vicon'

local time = require'unix'.time
local poll = require'unix'.poll
local logger = require'logger'
local log = flags.log~=0 and assert(logger.new('vicon', flags.home.."/logs"))

local skt = require'skt'
local transport = assert(skt.open{
                         port=vicon.BASE_PORT,
                         use_connect=false})

local function exit()
  if log then log:close() end
  transport:close()
end
racecar.handle_shutdown(exit)

local co_parse = coroutine.create(vicon.update)

local t0 = time()
local t_debug = t0
while racecar.running do
  local rc, ready = poll({transport.fd}, 1e3)
  if rc and ready then
    local pkt, status = transport:recv()
    local obj, err
    if pkt then
      status, obj, err = coroutine.resume(co_parse, pkt)
    end
    if not status then
      io.stderr:write(string.format("Error: [%s][%s]\n", tostring(status), tostring(obj)))
    elseif obj then
      log_announce(log, obj, "vicon")
    else
      io.stderr:write(string.format("Bad processing! [%s]\n", tostring(err)))
    end
  elseif rc then
    io.stderr:write("No data!\n")
  else
    io.stderr:write("uh oh\n")
  end
  local t_poll = time()
  local dt_debug = t_poll - t_debug
  if dt_debug > 1 then
    local info = jitter_tbl()
    io.write(table.concat(info, '\n'), '\n')
    t_debug = t_poll
  end
end
