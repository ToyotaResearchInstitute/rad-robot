#!/usr/bin/env luajit

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)

local coresume = require'coroutine'.resume
local max, min = require'math'.max, require'math'.min
local schar = require'string'.char
local unpack = unpack or require'table'.unpack

local has_logger, logger = pcall(require, 'logger')
local vesc = require'vesc'
local log_announce = racecar.log_announce
local log = has_logger and flags.log~=0
            and assert(logger.new('vesc', flags.home.."/logs"))

local fd_vesc, read, write
if flags.vesc then
  local stty = require'stty'
  local unix = require'unix'
  read = require'unix'.read
  write = require'unix'.write
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
  servo = false,
  velocity = false,
  sensor_request = false
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
  cmds.velocity = tonumber(obj.velocity) or 0
end

-- TODO: Put into VESC library
local coro_vesc = coroutine.create(vesc.update)
local pkt_req_values = schar(unpack(vesc.sensors()))
local function cb_loop()
  if not fd_vesc then
    return false, "No file descriptor"
  end
  -- Read to find data
  local data = read(fd_vesc)
  -- TODO: Check this...
  if data==-1 then
    print("VESC | Bad read")
  elseif type(data)=='string' then
    local status, obj, msg = coresume(coro_vesc, data)
    while status and obj do
      log_announce(log, obj, 'vesc')
      status, obj, msg = coresume(coro_vesc)
    end
  end
  -- Write commands
  local pkt_servo = vesc.servo_position(cmds.servo)
  if pkt_servo then
    write(fd_vesc, schar(unpack(pkt_servo)))
    cmds.servo = false
  end
  local pkt_velocity = vesc.duty_cycle(cmds.velocity)
  if pkt_velocity then
    write(fd_vesc, schar(unpack(pkt_velocity)))
    cmds.velocity = false
  end
  -- Save the commands in the log file
  log_announce(log, cmds, 'vesc')
  -- Ask for sensors again
  --write(fd_vesc, pkt_req_values)
  -- stty.drain(fd_vesc)
end

-- Listen at 100Hz
racecar.listen({control=cb_control}, 10, cb_loop)
