#!/usr/bin/env luajit
local racecar = require'racecar'
local flags = racecar.parse_arg(arg)

local razor_imu = require'razor_imu'
local unix = require'unix'

local f_imu = assert(io.open(flags.imu))
local fd_imu = assert(unix.fileno(f_imu))

local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log~=0
            and assert(logger.new('imu', racecar.ROBOT_HOME.."/logs"))

local log_announce = racecar.log_announce

local function exit()
  if log then log:close() end
  f_imu:close()
  f_imu = nil
  fd_imu = nil
end
racecar.handle_shutdown(exit)

local co_imu = coroutine.create(razor_imu.co_update)

local function on_imu(e)
  if e~=1 and f_imu then
    print("Reading", e)
    exit()
    return os.exit()
  end
  local pkt = unix.read(fd_imu)
  if not pkt then
    io.stderr:write('No pkt\n')
    return
  end
  local status, obj = coroutine.resume(co_imu, pkt)
  -- print("status", status)
  --[[
  local obj, err = razor_imu.update(pkt)
  if not obj then
    io.stderr:write(err, '\n')
    return
  end
  --]]
  log_announce(log, obj, "imu")
end

local fd_updates = {
  [fd_imu] = on_imu
}

racecar.listen{
  fd_updates = fd_updates,
}
exit()
