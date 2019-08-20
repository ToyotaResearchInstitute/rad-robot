#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)

local vector = require'vector'

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('plan', racecar.ROBOT_HOME.."/logs"))

-- Test the control library
local highway = require'highway'

-- Start configurations - this is a JSON file
local has_cjson, cjson = pcall(require, 'cjson')
local configuration = has_cjson and flags.config
if type(configuration) == 'string' and configuration:match"%.json$" then
  local f_conf = assert(io.open(configuration))
  configuration = cjson.decode(f_conf:read"*all")
  f_conf:close()
end

-- Using a 1/10th scale car, so divide each by 10 :P
local interstate95 = highway.new{
  length = 51.7, -- length in kilometers
  marker_interval = 0.10 -- marker intervals, in kilometers
}

local interstate76 = highway.new{
  length = 3.781958, -- length in kilometers
  marker_interval = 0.10 -- marker intervals, in kilometers
}

local highways = {
  ['i95'] = interstate95,
  ['i76'] = interstate76,
}

-- Set the speed limit
interstate95:add_event(0, "speed_limit", {
  meters_per_second = 2
})
-- Initialize the lanes
interstate95:add_event(0, "lane", {
  n = 3
})
-- At 10 meters (0.01 kilometers)
interstate95:add_event(0.010, "exit", {
  id = 1, -- Exit 1
  highway = {'i95', 505.25} -- the connecting highway, id and distance
})

-- Highway frame of reference (xmin, ymin, xlength, ylength)
-- Look 50 meters, behind (@ 1/10th scale) and 100m ahead
local viewBox_H = {
  -5, -2, 15, 6
}

local env = {
  viewBox = viewBox_H,
  highways = highways,
}

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  assert(log_announce(log, env, "planner"))
end
-- Update the pure pursuit
--------------------------

local function cb_debug(t_us)
  print("Env")
  for k, v in pairs(env) do
    print(k, v)
  end
end

local cb_tbl = {
  
}

local function exit()
  if log then log:close() end
  return 0
end
racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = 1000, -- 1 Hz loop
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
