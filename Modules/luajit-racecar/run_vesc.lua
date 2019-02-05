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
  io.stderr:write("!! No physical VESC\n")
  read = function() end
  write = function() end
end

local function exit()
  if log then log:close() end
  close(fd_vesc)
  fd_vesc = -1
  return 0
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
local function cb_control(inp)
  local rpm = tonumber(inp.velocity)
  local duty = tonumber(inp.duty)
  cmds.rpm = tonumber(rpm * vesc.RPM_PER_MPS)
  cmds.servo = steering2servo(tonumber(inp.steering) or 0)
end

local vel_min = 0.2
local vel_max = 1
local function parse_joystick(msg)
  if type(msg.buttons)~='table' or type(msg.axes)~='table' then
    return
  end
  cmds.servo = steering2servo(msg.axes[3] / -32767)
  local zero_to_one = msg.axes[2] / -32767
  cmds.duty = zero_to_one
  local mps = vel_max * zero_to_one
  if math.abs(mps) > vel_min then
    cmds.rpm = mps * vesc.RPM_PER_MPS
  else
    -- cmds.rpm = false
    cmds.rpm = 0
  end
end

-- TODO: Put into VESC library
local coro_vesc = coroutine.create(vesc.update)

-- Read to find data
local function update_read(e)
  if e~=1 and fd_vesc >= 0 then
    print("VESC | Bad read", e)
    exit()
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
  if pkt_rpm then
    write(fd_vesc, schar(unpack(pkt_rpm)))
    cmds.duty = false
  elseif pkt_duty then
    write(fd_vesc, schar(unpack(pkt_duty)))
    cmds.rpm = false
  end
  -- Save the commands in the log file
  log_announce(log, cmds, 'vesc')
  -- stty.drain(fd_vesc)
end

-- Listen at 100Hz
local cb_tbl = {
  control = cb_control,
  joystick = parse_joystick,
}
local fd_updates = {}
if fd_vesc then
  fd_updates[fd_vesc] = update_read
end

racecar.handle_shutdown(exit)
racecar.listen{
  channel_callbacks = cb_tbl,
  fd_updates = fd_updates,
  loop_rate = 10,
  fn_loop = cb_loop
}
exit()
