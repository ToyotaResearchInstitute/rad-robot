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
            and assert(logger.new('joystick', racecar.ROBOT_HOME.."/logs"))

local js = require'joystick'
local fd_js = assert(js.open(flags.js or "/dev/input/js0", true))

-- Read to find data
local function update_read(e)
--[[
  if e~=1 then
    print("Reading", e)
    js.close()
    fd_js = false
    return os.exit()
  end
--]]
  local axes = js.axis()
--  print('Axes', unpack(axes))
  --
  local buttons = js.button()
--  print('Buttons', unpack(buttons))
  return axes, buttons
end


local t_loop_last = 0
local function cb_loop(t_us)
  local dt = tonumber(t_us - t_loop_last) / 1e6
  if dt < 0.010 then
    return false, "Looping too fast"
  elseif not fd_js then
    return false, "No file descriptor"
  end
  t_loop_last = t_us

  local axes, buttons = update_read(e)

  -- Save the commands in the log file
  log_announce(log, {axes=axes, buttons=buttons}, 'joystick')
end



-- Listen at 100Hz
local cb_tbl = {
  houston = function() end
}
-- local fd_updates = {
--   [fd_vesc] = update_read
-- }
racecar.listen{
  channel_callbacks = cb_tbl,
  -- fd_updates = fd_updates,
  loop_rate = 50, -- 20Hz input
  fn_loop = cb_loop
}
