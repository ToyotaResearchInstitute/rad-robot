#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local log_announce = racecar.log_announce

local has_control, control = pcall(require, 'control')
local transform = require'transform'
local t0_us = require'unix'.time_us()
local vector = require'vector'

local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Simulation parameters
math.randomseed(123)
local dt_ms = 1.0e3/60 -- ms loop: Try 60 Hz
local dheading_max = math.rad(45) -- radians of front wheels
local wheel_base = 0.325

-- Start configurations - this is a JSON file
local has_cjson, cjson = pcall(require, 'cjson')
local configuration = has_cjson and flags.config
if type(configuration) == 'string' and configuration:match"%.json$" then
  local f_conf = assert(io.open(configuration))
  configuration = cjson.decode(f_conf:read"*all")
  f_conf:close()
end

-- Globally accessible variables
local veh_states = {}

--
local function cb_debug(t_us)
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

-- TODO: Process noise and measurement noise functions
local function noisy_pose(pose, dt)
  local pose_x, pose_y, pose_a = unpack(pose)
  -- Add some noise
  local dx_noise, dy_noise = unpack(vector.randn(2, 0.01 * dt, 0))
  local da_noise = unpack(vector.randn(1, math.rad(1) * dt, 0))
  -- Limit the noise
  local dx_noise_max = 0.01
  local dy_noise_max = 0.01
  local da_noise_max = math.rad(1)
  dx_noise = math.min(math.max(-dx_noise_max, dx_noise), dx_noise_max)
  dy_noise = math.min(math.max(-dy_noise_max, dy_noise), dy_noise_max)
  da_noise = math.min(math.max(-da_noise_max, da_noise), da_noise_max)
  -- Return a new pose
  local pose_w_noise = {
    pose_x + dx_noise,
    pose_y + dy_noise,
    pose_a + da_noise
  }
  -- pose_w_noise[3] = transform.mod_angle(pose_w_noise[2])
  return pose_w_noise
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
  -- pose_process[3] = transform.mod_angle(pose_process[2])
  -- Add noise to the process
  -- Only if moving!
  local use_noise = velocity > 0
  local pose_noisy = use_noise and noisy_pose(pose_process, dt) or pose_process
  return {
    pose = pose_noisy,
    controls = control_inp
  }

end

local function new_state()
  return {
    pose = {0,0,0}
  }
end

-- TODO: Configuration file should give this...
local function set_start_state(id_robot, state_robot)
  if id_robot=='tri1' then
    -- Roundabout
    state_robot.pose = {-1.0, 2.5, 0}
    -- Merge
    -- state_robot.pose = {-1.0, 2.5, 0}
    -- Left
    -- state_robot.pose =  {-1.0, 2.75, 0}
  else
    -- Roundabout
    state_robot.pose = {4.125, 2.5, math.rad(90)}
    -- Merge
    -- state_robot.pose = {-1.0, 3.5, 0}
    -- Left
    -- state_robot.pose = pose={0, 0, 0}
  end
end
local function cb_control(inp)
  local id_robot = inp.id
  if not id_robot then return false, "No ID to simulate" end
  -- Add this car
  -- TODO: Smarter way to initialize the state?
  -- TODO: Maybe randomly select until the pose is collision free, w.r.t to other poses?
  local state_robot = veh_states[id_robot]
  if not state_robot then
    state_robot = new_state()
    -- Special start state, based on the name
    set_start_state(id_robot, state_robot)
    veh_states[id_robot] = state_robot
  end
  -- TODO: Break out the steering, etc.
  state_robot.controls = inp
  -- Control the steering, this way
  -- local steering_pid = has_control and control.pid{
  --   r = 0
  -- }
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
  for id_rbt, state_now in pairs(veh_states) do
    -- Simulate a timestep
    local state_new = simulate_vehicle(state_now, dt)
    -- Update the internal state
    veh_states[id_rbt] = state_new
    -- Give noisy measurements
    local pose_noisy = noisy_pose(state_new.pose, dt)
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

local function exit()
  if log then log:close() end
  racecar.announce("control", {steering = 0, velocity = 0})
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
