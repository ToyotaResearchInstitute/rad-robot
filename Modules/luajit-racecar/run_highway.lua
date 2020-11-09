#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)

local vector = require'vector'

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('plan', racecar.ROBOT_HOME.."/logs"))

local env = {}

-- Start configurations - this is a JSON file
local has_cjson, cjson = pcall(require, 'cjson')
local configuration = has_cjson and flags.config
if type(configuration) == 'string' and configuration:match"%.json$" then
  local f_conf = assert(io.open(configuration))
  configuration = cjson.decode(f_conf:read"*all")
  f_conf:close()
else
  configuration = {}
end

-- Include the highway
local highway = require'highway'

local highways = {}
if type(configuration["highways"])=='table' then
  for hw_name, hw_config in pairs(configuration["highways"]) do
    highways[hw_name] = highway.new(hw_config)
  end
else
  -- TODO: Can listen to events that add/update highways
end
env.highways = highways
-- env.highways = {}
-- for k, v in pairs(highways) do
--   env.highways[k] = v:export()
-- end

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  log_announce(log, env, "planner")
end
-- Update the pure pursuit
--------------------------

local function cb_debug(t_us)
  print("Env")
  for k, v in pairs(env) do
    print(k, v)
  end
  -- for k, v in pairs(env.highways.events) do
  --   print(k, v)
  -- end
end

local cb_tbl = {
  
}

local function exit()
  if log then log:close() end
  return 0
end
-- racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = 1000, -- 1 Hz loop
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
