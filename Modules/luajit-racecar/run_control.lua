#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local id_robot = flags.id or racecar.HOSTNAME
assert(id_robot, "No name provided!")
math.randomseed(123)
local atan = require'math'.atan
local vector = require'vector'

local control = require'control'

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Initialize the system
racecar.init()

-- Globally accessible variables
local desired_path = flags.path or 'lane_outer'
-- TODO: Paths should come from a separate program
local pp_params
do
  local velocity = tonumber(flags.vel_lane) or 0.5
  local velocity_stddev = velocity * 0.1
  local vel_max = 0.75
  local vel_min = 0.2
  local lookahead = tonumber(flags.lookahead) or 0.325 -- 0.425, 0.65
  pp_params = {
    path = false,
    lookahead = lookahead,
    threshold_close = 0.25,
    leader = {false, false},
    velocity = velocity,
    velocity_stddev = velocity_stddev,
    velocity_max = vel_max,
    velocity_min = vel_min,
  }
end
local pp_control = false
local pp_result = false
--
local veh_poses = {}
-- Simulation parameters
local wheel_base = 0.3
-- Parameters for trajectory following
local dt = 0.1
--
local function cb_debug(t_us)
  local pose_rbt = veh_poses[id_robot]
  if not pose_rbt then return end
  local px, py, pa = unpack(pose_rbt)
  local info = {
    string.format("Path: %s", desired_path),
    string.format("Pose: x=%.2fm, y=%.2fm, a=%.2f°", px, py, math.deg(pa)),
    string.format("Leader: %s [%s]", unpack(pp_params.leader)),
  }
  if pp_result then
    table.insert(info, string.format("Control: %.2f m/s, %.2f°",
      pp_result.velocity, math.deg(pp_result.steering)))
  else
    table.insert(info, "Control not available")
  end
  return table.concat(info, "\n")
end

local function find_lead(path_lane, id_path)
  if type(path_lane)~='table' then
    return false, "Bad path: "..type(path_lane)
  end
  -- Check if in my lane
  local in_my_lane = {}
  for name_veh, pose_veh in pairs(veh_poses) do
    if name_veh ~= id_robot then
      local info = control.find_in_path(pose_veh, path_lane, 0.25)
      -- Check if they are in our lane
      if info then
        table.insert(in_my_lane, {name_veh, info})
      end
    end
  end
  if #in_my_lane == 0 then
    return false, "Nobody in my lane"
  end
  local name_lead, d_lead = false, math.huge
  -- local pose_rbt = veh_poses[id_robot]
  -- local p_x, p_y, p_a = unpack(pose_rbt)
  -- print("Monitoring my lane: ", table.concat(in_my_lane, ", "))
  for _, name_info in ipairs(in_my_lane) do
    local name_veh, info_veh = unpack(name_info)
    local id_path_other = info_veh.id_in_path
    local d_id = id_path_other - id_path
    local d_rel = d_id * path_lane.ds
    -- Loop around if closed
    if path_lane.closed then
      if id_path_other < id_path then
        id_path_other = id_path_other + #path_lane.points
      else
        id_path_other = id_path_other - #path_lane.points
      end
      local d_id2 = id_path_other - id_path
      local d_rel2 = d_id2 * path_lane.ds
      if math.abs(d_rel2) < math.abs(d_rel) then
        d_rel = d_rel2
      end
    end
    if d_rel > 0 and d_rel < d_lead then
      d_lead = d_rel
      name_lead = name_veh
    end
  end
  return name_lead, d_lead
end

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  local pose_rbt = veh_poses[id_robot]
  -- Stop if no pose
  if not pose_rbt then
    local control_inp = {id = id_robot, steering = 0, velocity = 0}
    log_announce(log, control_inp, "control")
    return
  end
  if not pp_control then
    return false, "No controller"
  end
  -- Find our control policy
  local result, err = pp_control(pose_rbt)
  pp_result = result
  if not result then
    return false, err
  end
  if result.err then
    return false, result.err
  end
  -- Ensure we add our ID to the result
  result.id = id_robot
  result.pathname = desired_path
  -- Save the pose, just because
  result.pose = pose_rbt
  local ratio = 1
  --------------------------------
  -- Check the obstacles around us
  local name_lead, d_lead = find_lead(pp_params.path, result.id_path)
  -- Allow for printing
  pp_params.leader = {name_lead, d_lead}
  -- Slow for a lead vehicle
  if name_lead then
    -- print("Lane leader | ", name_lead, d_lead)
    local d_stop = 1.5 * wheel_base
    local d_near = 3 * wheel_base
    if d_lead < d_near then
      ratio = (d_lead - d_stop) / (d_near - d_stop)
      -- Bound the ratio
      ratio = math.max(0, math.min(ratio, 1))
    end
  end
  --------------------------------

  -- When looping, update the velocity
  local vel_sample = pp_params.velocity
  -- Just update all the time, now
  if true or result.id_path == 1 then
    local sample = vector.randn(1, pp_params.velocity_stddev, pp_params.velocity)
    -- Bound the sample
    vel_sample = math.max(pp_params.velocity_min, math.min(sample[1], pp_params.velocity_max))
  end

  if result.done then
    result.steering = 0
    result.velocity = 0
  else
    -- Set the steering based on the car dimensions
    local steering = atan(result.kappa * wheel_base)
    result.steering = steering
    -- TODO: Set the speed based on curvature? Lookahead point's curvature?
    result.velocity = vel_sample * ratio
  end

  -- Broadcast
  log_announce(log, result, "control")
end
-- Update the pure pursuit
--------------------------

-------------------
-- Update the vehicle poses
local function vicon2pose(vp)
  return {
    vp.translation[1] / 1e3,
    vp.translation[2] / 1e3,
    vp.rotation[3]
  }
end
local last_frame = -math.huge
local function parse_vicon(msg)
  -- Check that the data is not stale
  -- TODO: Stale for each ID...
  local frame = msg.frame
  msg.frame = nil
  if frame <= last_frame then
    return false, "Stale data"
  end
  last_frame = frame
  -- Find the pose for each robot
  for id, vp in pairs(msg) do
    veh_poses[id] = vicon2pose(vp)
  end
end
-- Update the poses
-------------------

local function parse_plan(msg)
  local my_path = msg.paths[desired_path]
  if type(my_path)=='table' then
    -- We've found our table
    if pp_params.path ~= my_path or not pp_control then
      -- KD Tree of points
      local pt_tree = control.generate_kdtree(my_path.points)
      my_path.tree = pt_tree
      -- Update the parameters
      pp_params.path = my_path
      pp_control = assert(control.pure_pursuit(pp_params))
    end
  end
end

local cb_tbl = {
  vicon = parse_vicon,
  risk = parse_plan
}

local function exit()
  if log then log:close() end
  log_announce(log, {id = id_robot, steering = 0, velocity = 0}, "control")
  return 0
end
racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = 100, -- 100ms loop
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
