#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local log_announce = racecar.log_announce

local transform = require'transform'
local t0_us = require'unix'.time_us()
local vector = require'vector'

local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Simulation parameters
math.randomseed(123)
local dt_sim = 1 / 60
local dt_ms = 1.0e3 / 60 -- ms loop: Try 60 Hz
local dheading_max = math.rad(45) -- radians of front wheels
local wheel_base = 0.325

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

-- Input: pose
-- Output: vicon table
local function pose2vicon(p)
  -- Just on the z=0 plane
  return {
    rotation = {0, 0, p[3]},
    translation = {1e3 * p[1], 1e3 * p[2], 0},
  }
end

-- Globally accessible variables
local veh_states = {}

local function new_state(state0)
  -- Copy
  local px, py, pa
  if state0 and state0.pose then
    px, py, pa = unpack(state0.pose)
  else
    px, py, pa = 0, 0, 0
  end
  return {
    pose = {px, py, pa},
    controls = {steering=0, velocity=0}
  }
end

--
local function cb_debug(t_us, cnt)

  -- Broadcast the viewbox
  -- Highway frame of reference (xmin, ymin, xlength, ylength)
  local info_debug = {
    viewBox = configuration.viewBox or {-2, -2, 8, 4},
    reference_vehicle = configuration.reference_vehicle
  }
  racecar.announce("debug", info_debug, cnt)
  local info = {
  string.format("Simulation time: %.2f", tonumber(t_us - t0_us)/1e6)
  }
  for id_rbt, state_rbt in pairs(veh_states) do
    local px, py, pa = unpack(state_rbt.pose)
    table.insert(info,
      string.format("Robot: %s | x=%.2fm, y=%.2fm, a=%.2f°",
        id_rbt, px, py, math.deg(transform.mod_angle(pa)))
    )
  end
  return table.concat(info, "\n")
end

local function simulate_vehicle(state, dt)
  local control_inp = state.controls
  local steering = control_inp.steering
  local velocity = control_inp.velocity
  local pose_x, pose_y, pose_a = unpack(state.pose)
  -- Kinematic model propagation
  steering = math.min(math.max(-dheading_max, steering), dheading_max)
  local dpose_a = (velocity * dt) / wheel_base * math.tan(steering)
  local dpose_x, dpose_y = transform.rot2D(velocity * dt, 0, pose_a)
  -- Give the new pose, deterministically
  local pose_process = {
    pose_x + dpose_x,
    pose_y + dpose_y,
    pose_a + dpose_a
  }
  -- Add noise to the process
  -- Only if moving!
  local use_noise = velocity > 0
  local pose_noisy
  if use_noise then
    pose_noisy = vector.randn_pose(
      {0.01 * dt, 0.01 * dt, math.rad(1) * dt},
      pose_process)
  else
    pose_noisy = pose_process
  end
  return {
    pose = pose_noisy,
    controls = control_inp
  }
end

local function cb_control(inp, ch, t_us)
  local id_robot = inp.id
  if not id_robot then return false, "No ID to simulate" end
  -- Add this car
  -- TODO: Smarter way to initialize the state?
  -- TODO: Maybe randomly select until the pose is collision free, w.r.t to other poses?
  local state_robot = veh_states[id_robot]
  if not state_robot then
    -- Special start state, based on the name
    local config_initial = configuration["initialization"]
    if type(config_initial)=='table' then
      config_initial = config_initial[id_robot] or config_initial[""]
    end
    state_robot = new_state(config_initial)
    veh_states[id_robot] = state_robot
  end
  -- TODO: Break out the steering, etc.
  state_robot.controls = inp
  -- Control the steering, this way
  -- local steering_pid = has_control and control.pid{
  --   r = 0
  -- }
end

--------------------------
-- Update the pure pursuit
local count_frame = 0
-- local t_last_us
local function cb_loop(t_us)
  local msg_vicon = {
    frame = count_frame
  }
  -- Simulate and publish this timestep
  local dt_us = t_us - (t_last_us or t_us)
  t_last_us = t_us
  local dt = tonumber(dt_us) / 1e6
  local dt_diff = dt - dt_sim
  count_frame = count_frame + 1
  for id_rbt, state_now in pairs(veh_states) do
    -- Simulate a timestep
    local state_new = simulate_vehicle(state_now, dt_sim)
    -- Update the internal state
    veh_states[id_rbt] = state_new
    -- Give noisy measurements
    local pose_noisy = vector.randn_pose(
      {0.01 * dt_sim, 0.01 * dt_sim, math.rad(1) * dt_sim},
      state_new.pose)
    -- Set the publishing message
    msg_vicon[id_rbt] = pose2vicon(pose_noisy)
  end
  -- Publish or return the vicon message
  log_announce(log, msg_vicon, "vicon")
end
-- Update the pure pursuit
--------------------------

local cb_tbl = {
  control = cb_control,
}

local function entry()
  -- Special start state, based on the name
  local config_initial = configuration["initialization"]
  if type(config_initial)=='table' then
    for id_robot, state0 in pairs(config_initial) do
      if #id_robot > 0 then
        veh_states[id_robot] = new_state(state0)
      end
    end
  end
end

local function exit()
  if log then log:close() end
  return 0
end
-- racecar.handle_shutdown(exit)

entry()
racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = dt_ms,
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
