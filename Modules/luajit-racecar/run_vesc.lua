#!/usr/bin/env luajit

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
racecar.init()

local coresume = require'coroutine'.resume
local max, min = require'math'.max, require'math'.min
local schar = require'string'.char
local unpack = unpack or require'table'.unpack

local has_logger, logger = pcall(require, 'logger')
local log_announce = racecar.log_announce
local log = has_logger and flags.log~=0
            and assert(logger.new('vesc', racecar.ROBOT_HOME.."/logs"))

local vesc = require'vesc'

local fd_vesc, read, write, close
if flags.vesc then
  local stty = require'stty'
  local unix = require'unix'
  read = require'unix'.read
  write = require'unix'.write
  close = require'unix'.close
  fd_vesc = assert(unix.open(flags.vesc, unix.O_RDWR + unix.O_NOCTTY + unix.O_NONBLOCK))
  assert(fd_vesc > 0, "Bad File descriptor")
  stty.raw(fd_vesc)
  stty.serial(fd_vesc)
  stty.speed(fd_vesc, 115200)
else
  read = function() end
  write = function() end
end

local cmds = {
  servo = 0,
  duty = 0,
  sensor_request = true
}

local steer_max = math.pi / 4
local function steering2servo(steering)
  -- Clip
  steering = max(-steer_max, min(steering, steer_max))
  -- Put into range of [0, 1] with 0.5 center (sign flipped, too)
  return steering / (-2 * steer_max) + 0.5
end
local function cb_control(obj)
  cmds.servo = steering2servo(tonumber(obj.steering) or 0)
  cmds.duty = tonumber(obj.duty)
  cmds.rpm = tonumber(obj.rpm)
  -- print("cb control")
end

-- TODO: Put into VESC library
local coro_vesc = coroutine.create(vesc.update)

-- Read to find data
local function update_read(e)
  if e~=1 and fd_vesc >= 0 then
    print("Reading", e)
    close(fd_vesc)
    fd_vesc = -1
    return os.exit()
  end
  -- TODO: Check the type of event:
  -- e.g. in case the device was unplugged
  local data = read(fd_vesc)
  -- TODO: Check this...
  if data==-1 then
    return false, "Bad read"
  elseif type(data)~='string' then
    return false, "Weird read: "..type(data)
  end
  local status, obj, msg = coresume(coro_vesc, data)
  while status and obj do
    log_announce(log, obj, 'vesc')
    status, obj, msg = coresume(coro_vesc)
  end
end

local pkt_req_values = schar(unpack(vesc.sensors()))
local t_loop_last = 0
local function cb_loop(t_us)
  local dt = tonumber(t_us - t_loop_last) / 1e6
  if not fd_vesc then
    return false, "No file descriptor"
  end
  t_loop_last = t_us
  -- update_read()
  -- Ask for sensors again
  write(fd_vesc, pkt_req_values)
  -- Write commands
  local pkt_servo = vesc.servo_position(cmds.servo)
  if pkt_servo then
    write(fd_vesc, schar(unpack(pkt_servo)))
    cmds.servo = false
  end
  local pkt_duty = cmds.duty and vesc.duty_cycle(cmds.duty)
  local pkt_rpm = cmds.rpm and vesc.rpm(cmds.rpm)
  if pkt_duty then
    write(fd_vesc, schar(unpack(pkt_duty)))
    cmds.duty = false
  elseif pkt_rpm then
    write(fd_vesc, schar(unpack(pkt_rpm)))
    cmds.rpm = false
  end
  -- Save the commands in the log file
  log_announce(log, cmds, 'vesc')
  -- stty.drain(fd_vesc)
end

-- Listen at 100Hz
local cb_tbl = {
  control = cb_control
}
local fd_updates = {}
if fd_vesc then
  fd_updates[fd_vesc] = update_read
end
racecar.listen{
  channel_callbacks = cb_tbl,
  fd_updates = fd_updates,
  loop_rate = 10,
  loop_fn = cb_loop
}

