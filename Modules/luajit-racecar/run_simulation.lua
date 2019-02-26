#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local log_announce = racecar.log_announce

local DEBUG_ANNOUNCE = os.getenv("ANNOUNCE") or flags.announce
math.randomseed(123)

local transform = require'transform'
-- local usleep = require'unix'.usleep
local vector = require'vector'

local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Globally accessible variables
local veh_states = {}
local veh_controls = {}

-- Simulation parameters
local dt_ms = 100 -- 50 -- ms loop
local dheading_max = math.rad(45) -- radians of front wheels
local wheel_base = 0.3

--
local function cb_debug(t_us)
  local info = {
  string.format("Simulation time: %.2f", tonumber(t_us)/1e6)
  }
  for id_rbt, state_rbt in pairs(veh_states) do
    local px, py, pa = unpack(state_rbt.pose)
    table.insert(info,
      string.format("Robot: %s | x=%.2fm, y=%.2fm, a=%.2f°",
        id_rbt, px, py, math.deg(pa))
    )
  end
  return table.concat(info, "\n")
end

local function noisy_pose(pose, dt)
  local pose_x, pose_y, pose_a = unpack(pose)
  -- Add some noise
  local dnoise_x, dnoise_y = unpack(vector.randn(2, 0.01 * dt, 0))
  local dnoise_a = unpack(vector.randn(1, math.rad(1) * dt, 0))
  return {
    pose_x + dnoise_x,
    pose_y + dnoise_y,
    transform.mod_angle(pose_a + dnoise_a)
  }
end

local function simulate_vehicle(state, control_inp, dt, use_noise)
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
    transform.mod_angle(pose_a + dpose_a)
  }
  -- Add noise to the process
  local pose_noisy = use_noise and noisy_pose(pose_process, dt) or pose_process
  return {
    pose = pose_noisy
  }

end

-- Simulate per the robot
local function start_states(id_robot)
  if id_robot=='tri1' then
    -- Roundabout
    return {pose={-1.0, 2.5, 0}}
    -- Merge
    -- return {pose={-1.0, 2.5, 0}}
    -- Left
    -- return {pose={-1.0, 2.75, 0}}
  else
    -- Roundabout
    return {pose={4.125, 2.5, math.rad(90)}}
    -- Merge
    -- return {pose={-1.0, 3.5, 0}}
    -- Left
    -- veh_states[id_robot] = {pose={0, 0, 0}}
  end
end
local function cb_control(inp)
  local id_robot = inp.id
  if not id_robot then return false, "No ID to simulate" end
  veh_controls[id_robot] = inp
  -- Add this car
  -- TODO: Smarter way to initialize the state?
  -- TODO: Maybe randomly select until the pose is collision free, w.r.t to other poses?
  local veh_state = veh_states[id_robot]
  if not veh_state then
    veh_states[id_robot] = start_states(id_robot)
  elseif veh_state.pose[1] > 4.5 then
    local state_new = start_states(id_robot)
    -- Add some noise, for merge not to be the same
    local dnoise_x = math.abs(unpack(vector.randn(1, 1, 0)))
    state_new.pose[1] = state_new.pose[1] - dnoise_x
    veh_states[id_robot] = state_new
  end
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

--------------------------
-- Update the pure pursuit
local count_frame = 0
local t_last_us
local function cb_loop(t_us)
  -- Simulate and publish this timestep
  local dt_us = t_us - (t_last_us or t_us)
  t_last_us = t_us
  local dt = tonumber(dt_us) / 1e6
  local msg_vicon = {
    frame = count_frame
  }
  count_frame = count_frame + 1
  for id_rbt, ctrl_inp_rbt in pairs(veh_controls) do
    -- Simulate a timestep
    local state_now = veh_states[id_rbt]
    local use_noise = ctrl_inp_rbt.pathname ~= 'driveway'
    local state_new = simulate_vehicle(state_now, ctrl_inp_rbt, dt, use_noise)
    -- Update the internal state
    veh_states[id_rbt] = state_new
    -- Give noisy measurements
    local pose_noisy = noisy_pose(state_new.pose, dt)
    -- Set the publishing message
    msg_vicon[id_rbt] = pose2vicon(pose_noisy)
  end
  -- Publish or return the vicon message
  if DEBUG_ANNOUNCE then
    log_announce(log, msg_vicon, "vicon")
    -- Wait a touch when simulating?
    -- usleep(1e4)
  else
    return msg_vicon
  end
end
-- Update the pure pursuit
--------------------------

local cb_tbl = {
  control = cb_control,
}

local function exit()
  if log then log:close() end
  if DEBUG_ANNOUNCE then
    racecar.announce("control", {steering = 0, velocity = 0})
  end
  return 0
end
racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = dt_ms,
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
