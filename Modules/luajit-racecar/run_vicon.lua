#!/usr/bin/env luajit
local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
racecar.init()
local log_announce = racecar.log_announce

local vicon = require'vicon'

local logger = require'logger'
local log = flags.log~=0 and assert(logger.new('vicon', racecar.ROBOT_HOME.."/logs"))

local skt = require'skt'
local skt_vicon = assert(skt.open{
                         port=vicon.BASE_PORT,
                         use_connect=false})

local function exit()
  if log then log:close() end
  skt_vicon:close()
  skt_vicon = nil
end
-- racecar.handle_shutdown(exit)

local function on_vicon(e)
  if e~=1 and skt_vicon then
    print("Reading", e)
    exit()
    return os.exit()
  end
  local pkt, status = skt_vicon:recv()
  if not pkt then
    io.stderr:write(status, '\n')
    return
  end
  local obj, err = vicon.parse(pkt)
  if not obj then
    io.stderr:write(err, '\n')
    return
  end
  log_announce(log, obj, "vicon")
end

local fd_updates = {
  [skt_vicon.fd] = on_vicon
}

racecar.listen{
  fd_updates = fd_updates,
}
exit()
