#!/usr/bin/env luajit

-- TODO: log_announce the flags
local racecar = require'racecar'
local flags = require'racecar'.parse_arg(arg)
local log_announce = require'racecar'.log_announce
local jitter_tbl = require'racecar'.jitter_tbl

local DEBUG = (flags.debug == 0 and math.huge) or flags.debug or 1
local RACECAR_HOME = os.getenv"RACECAR_HOME" or '.'

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
  flags.imu = flags.imu or devices.imu
  flags.vesc = flags.vesc or devices.vesc
  flags.gps = flags.gps or devices.ublox
  flags.js = flags.js or io.popen"ls -1 /dev/input/js*":read"*line"
end
populate_dev()
io.write("IMU:\t", tostring(flags.imu), "\n")
io.write("VESC:\t", tostring(flags.vesc), "\n")
io.write("Joystick:\t", tostring(flags.js), "\n")
io.write("GPS:\t", tostring(flags.gps), "\n")

if flags.log_dir then
  local status = os.execute("test -d "..flags.log_dir)
  flags.log_dir = status and status~=0 and flags.log_dir
end
flags.log_dir = flags.log_dir or (RACECAR_HOME.."/logs")

-- Modules
local stty = require'stty'
local unix = require'unix'
local utime = require'unix'.time_us
local time = require'unix'.time
local has_js, js = pcall(require, 'joystick')
if not has_js then io.stderr:write"No Joystick Support\n" end
local has_imu, razor_imu = pcall(require, 'razor_imu')
has_imu = has_imu and flags.imu
if not has_imu then io.stderr:write"No IMU Support\n" end
local has_gps, nmea = pcall(require, 'nmea')
has_gps = has_gps and flags.gps
if not has_gps then io.stderr:write"No GPS Support\n" end
local has_signal, signal = pcall(require, 'signal')
if not has_signal then io.stderr:write"No signal Support\n" end
local has_vesc, vesc = pcall(require, 'vesc')
has_vesc = has_vesc and flags.vesc
if not has_vesc then io.stderr:write"No vesc support\n" end
local has_logger, logger = pcall(require, 'logger')
if not has_logger then io.stderr:write"No logger Support\n" end
local has_cofsm, cofsm = pcall(require, 'cofsm')
if not has_cofsm then io.stderr:write"No cofsm Support\n" end

-- Global variables
local sensors_latest = {t = time()}
local next_sensors = false
local fd_vesc, f_imu, f_gps, fd_js
local log
local t_entry, t_exit, t_update, t
local t_sensor0, t_sensor1
-- Saving the sensor information
local fds = {}
local coros = {}
local names = {}
local tsensors = {}
local botFsm = has_cofsm and cofsm.new{
  {'botStop', botStop},
  {'botStop', 'go', 'botGo'},
  {'botGo', 'stop', 'botStop'},
}
--
local pkt_req_values = has_vesc and
  string.char(unpack(vesc.sensors()))
local JOYSTICK_TO_MILLIAMPS = 40000 / -32767
local JOYSTICK_TO_SERVO = 1 / (2 * 32767)
local JOYSTICK_TO_DUTY = 11 / -32767
local JOYSTICK_TO_RPM = 30000 / -32767
--
local motor_modes = {'pwm', 'mA', 'rpm'}
local motor_mode = motor_modes[1]
if flags.motor_mode then
  for _, mode in ipairs(motor_modes) do
    if mode == flags.motor_mode then
      motor_mode = mode
      print("Manually setting motor_mode:", motor_mode)
      break
    end
  end
end

local joystick_to_pkt = {
  mA = function(pedal)
    pedal = pedal * JOYSTICK_TO_MILLIAMPS
    return vesc.current(pedal)
  end,
  pwm = function(pedal)
    pedal = pedal * JOYSTICK_TO_DUTY
    if math.abs(pedal) < 3 then pedal = 0 end
    return vesc.duty_cycle(pedal)
  end,
  rpm = function(pedal)
    pedal = pedal * JOYSTICK_TO_RPM
    -- io.stderr:write("RPM:", pedal,"\n")
    local pkt = vesc.rpm(pedal)
    -- io.stderr:write("RPM:", type(pkt),"\n")
    return pkt
  end,
}

local function add_sensor(name, fd, coro)
  name = tostring(name)
  table.insert(fds, fd)
  table.insert(coros, type(coro)=='thread' and coro or false)
  table.insert(names, name)
  tsensors[name] = {}
  return #fds
end

local function process_sensor(ind, t)
  local coro = coros[ind]
  local status = coroutine.status(coro)
  if status ~= 'suspended' then return false, status end
  local data, obj
  local name = names[ind]
  -- vesc, gps, imu, js
  data = unix.read(fds[ind])
  if not data or data == -1 then
    io.stederr:write("Disconnected from "..tostring(name).."!\n")
    log_announce(log, {disconnect=name}, 'racecar_errors')
  end
  -- Keep asking for data
  local n = 0
  repeat
    local status, ret = coroutine.resume(coro, data)
    if not status then
      io.stderr:write(ret, '\n')
      return status, ret
    elseif ret then
      obj = ret
      obj.t = obj.t or t
      log_announce(log, obj, name)
      sensors_latest[name] = obj
      table.insert(tsensors[name], t)
      n = n + 1
    end
    data = nil
  until not ret
  return n
end

local function real_sensors()
  local t_vesc_req, vesc_ms = -math.huge, 1 / 50
  local dt_timeout = 1 / 50
  local t_sensor = -math.huge
  while true do
    t_sensor_last = t_sensor
    t_sensor = time()
    local dt_vesc = t_sensor - t_vesc_req
    -- Ask for VESC sensors, read later
    if fd_vesc and (dt_vesc > vesc_ms) then
      t_vesc_req = t_sensor
      unix.write(fd_vesc, pkt_req_values)
      -- stty.drain(fd_vesc)
    end
    -- Affect the global table
    sensors_latest = {t = t_sensor}
    local n_response = 0
    repeat
      -- Poll every 2 milliseconds (500Hz)
      local rc, ready = unix.poll(fds, 2)
      local t_poll = time()
      local dt_real = t_poll - t_sensor
      if not rc then
        io.stderr:write(string.format(
          "Bad poll: %s\n", tostring(ready)))
        break
      elseif ready then
        for _, ind in ipairs(ready) do
          n_response = n_response + (process_sensor(ind, t_poll) or 0)
        end
      end
    until dt_real >= dt_timeout
    coroutine.yield(n_response)
  end
end

local function next_action()
  print('Entry', 'ok')
  local steer, pedal = 0, 0
  local ret, pkt_pedal, pkt_steer
  local evt = false
  local t_last = -math.huge
  while coroutine.yield(evt) do
    -- print('Update', 'go', sensors_latest)
    -- local t = time()
    local js_inp = sensors_latest.joystick
    if js_inp then
      steer = tonumber(js_inp.steer) or steer
      steer = (steer * JOYSTICK_TO_SERVO) + 0.5
      pedal = tonumber(js_inp['pedal_'..motor_mode]) or pedal
    end
    if not (fd_vesc and fd_js) then
      evt = 'stop'
    else
      pkt_pedal = joystick_to_pkt[motor_mode](pedal)
      pkt_steer = vesc.servo_position(steer)
      unix.write(fd_vesc, string.char(unpack(pkt_pedal)))
      unix.write(fd_vesc, string.char(unpack(pkt_steer)))
      --stty.drain(fd_vesc)
      log_announce(log, {
        mode = motor_mode,
        pedal = pedal,
        steer = steer
      }, 'command')
    end
  end
  print('Exit', 'woo')
  return 'bye'
end
if botFsm then
  botFsm:add_coro('botGo', next_action)
  botFsm:enter('botGo')
else
  next_action = coroutine.create(next_action)
  coroutine.resume(next_action, true)
end

local function exit()
  t_exit = time()
  io.stderr:write("Shutting down... ", t_exit, "\n")
  if log then
    io.stderr:write("Close log file [", tostring(log.fname), "] ... ")
    log:close()
    io.stderr:write("Done!\n")
  end
  if f_imu then
    io.stderr:write("Close IMU... ")
    f_imu:close()
    io.stderr:write("Done!\n")
  end
  if f_gps then
    io.stderr:write("Close GPS... ")
    f_gps:close()
    io.stderr:write("Done!\n")
  end
  if fd_vesc then
    io.stderr:write("Close VESC... ")
    unix.close(fd_vesc)
    io.stderr:write("Done!\n")
  end
  if has_js then
    io.stderr:write("Close Joystick... ")
    -- TODO: Must fix the joystick module with poll
    -- js.close()
    io.stderr:write("Done!\n")
  end
end

local function entry()
  t_entry, t = time()
  io.stderr:write("Entry: ", t_entry, "\n")

  -- Check if replaying data
  local is_replay = type(flags.replay)=='string'
  -- next_sensors function will replay data
  next_sensors = (not is_replay) and coroutine.create(real_sensors)
  if is_replay then
    assert(has_logger, "Cannot replay, due to missing logger library")
    next_sensors = coroutine.create(replay_sensors)
    io.stderr:write("Opening file: ", flags.replay, "\n")
    coroutine.resume(next_sensors, flags.replay)
    return
  elseif has_logger and flags.log~=0 then
    log = assert(logger.new('racecar', flags.log_dir))
    -- Save the execution flags
    log_announce(log, flags, 'racecar_flags')
  else
    io.stderr:write"Not logging...\n"
  end

  -- Open the VESC motor controller
  if has_vesc then
    io.stderr:write("Opening VESC: ", flags.vesc, "\n")
    fd_vesc = assert(unix.open(flags.vesc,
      unix.O_RDWR + unix.O_NOCTTY + unix.O_NONBLOCK))
    stty.raw(fd_vesc)
    stty.serial(fd_vesc)
    stty.speed(fd_vesc, 115200)
    stty.flush(fd_vesc)
    add_sensor('vesc', fd_vesc, coroutine.create(vesc.update))
  end

  -- Open the imu
  if has_imu then
    f_imu = assert(io.open(flags.imu))
    local fd_imu = assert(unix.fileno(f_imu))
    add_sensor('imu', fd_imu, coroutine.create(razor_imu.update))
  end

  -- Open the gps
  if has_gps then
    f_gps = assert(io.open(flags.gps))
    local fd_gps = assert(unix.fileno(f_gps))
    add_sensor('gps', fd_gps, coroutine.create(nmea.update))
  end

  -- Open the joystick
  if has_js then
    fd_js = js.open(flags.js)
    has_js = type(fd_js)=='number'
    if has_js then
      add_sensor('joystick', fd_js, coroutine.create(function(data)
        local steer_axid, pedal_axid = 3, 2 -- 1, 4
        local mmKey = "pedal_"..motor_mode
        while true do
          local obj
          if data then
            local axes, btns
            if (#data%8)==0 then
              for i=1,#data-7, 8 do
                axes, btns = js.parse(data:sub(i, i+7))
              end
            end
            obj = axes and {
              [mmKey] = axes[pedal_axid],
              steer = axes[steer_axid]
            }
          end
          data = coroutine.yield(obj)
        end
      end))
    end
  end

end

local function update()
  local t_update = time()

  local ok_sensors, n_sensors = coroutine.resume(next_sensors)
  if not ok_sensors then
    io.stderr:write(tostring(n_sensors), "\n")
    return false
  end

  if n_sensors > 0 then
    t_sensor0 = t_sensor0 or sensors_latest.t
    t_sensor1 = sensors_latest.t
  end
  -- Send commands to the robot
  if botFsm then
    status, evt = botFsm:update()
    local ret = status and botFsm:dispatch(evt)
    -- print(status, evt, ret)
  else
    coroutine.resume(next_action, true)
  end

  -- Debug data
  local dt_loop = time() - t_update
  local dt_debug = t_update - (t_debug or -math.huge)
  if dt_debug > DEBUG then
    t_debug = t_update
    local info = jitter_tbl()
    if #info > 0 then
      io.write(table.concat(info, "\n"), "\n")
    end
  end
end

racecar.handle_shutdown(exit)
entry()
while racecar.running do
  local evt = update()
  if evt == false then running = false end
end
exit()

local dt_sensor = t_sensor1 and (t_sensor1 - t_sensor0)
io.stdout:write(string.format("Duration: %f seconds\n", dt_sensor or (0 / 0)))
