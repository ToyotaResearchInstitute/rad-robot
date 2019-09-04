#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local id_robot = flags.id or racecar.HOSTNAME
assert(id_robot, "No name provided!")
math.randomseed(123)
local atan = require'math'.atan
local fabs = require'math'.abs
local sqrt = require'math'.sqrt
local mod_angle = require'transform'.mod_angle
local vector = require'vector'

local control = require'control'
local highway = require'highway'
local path = require'path'

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Initialize the system
racecar.init()

-- Globally accessible variables
local desired_path = flags.path or 'lane_outerA+1' -- TODO: Should simply find the nearest lane
local next_path = false -- Needs map information
local path_list = {} -- Path list now and into the future. Should hold path_rollout_time seconds
local path_rollout_time = 5 -- How far ahead to look given the speed limit of each path
local houston_event = false

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
local pp_err = nil
--
local planner_state = false
local veh_poses = {}
-- Simulation parameters
local wheel_base = 0.325
--
local function cb_debug(t_us)
  local pose_rbt = veh_poses[id_robot]
  if not pose_rbt then return end
  local px, py, pa = unpack(pose_rbt)
  local info = {
    string.format("Path: %s | Next: %s", desired_path, next_path),
    string.format("Pose: x=%.2fm, y=%.2fm, a=%.2f°", px, py, math.deg(pa)),
    string.format("Leader: %s [%s]", unpack(pp_params.leader)),
  }
  if type(pp_result)=='table' then
    table.insert(info, string.format("Control: %.2f m/s, %.2f°",
      pp_result.velocity, math.deg(pp_result.steering)))
  else
    table.insert(info, "Control not available: "..tostring(pp_err))
  end
  return table.concat(info, "\n")
end

------------------------------------------
-- Utilities
------------------------------------------
local function find_lead(veh_lanes, path_lane, id_path)
  local name_lead, d_lead = false, math.huge
  -- local pose_rbt = veh_poses[id_robot]
  -- local p_x, p_y, p_a = unpack(pose_rbt)
  -- print("Monitoring my lane: ", table.concat(in_my_lane, ", "))
  for name_veh, info_veh in ipairs(veh_lanes) do
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
------------------------------------------
-- Utilities
------------------------------------------

local function update_path()
  if not planner_state then
    return false, "No planner information"
  end
  if not desired_path then
    -- TODO: Find the nearest path
    print("Finding nearest path...")
    return false, "No desired path"
  end
  if planner_state.paths then
    local my_path = planner_state.paths[desired_path]
    if not my_path then
      return false, "Desired path not found in the planner"
    end
    return planner.wrap(my_path)
  elseif planner_state.highways then
    -- TODO: Do not create too often
    local hw = planner_state.highways[desired_path]
    return highway.wrap(hw)
  end
end

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  local pose_rbt = veh_poses[id_robot]
  -- Stop if no pose
  if not pose_rbt then
    local control_inp = {id = id_robot, steering = 0, velocity = 0}
    log_announce(log, control_inp, "control")
    return false, "No pose"
  end

  -- Find our plan
  local my_path
  my_path, pp_err = update_path()
  if not my_path then
    return false, pp_err
  end

  -- Look ahead
  if planner_state.transitions and not next_path then
    local available = planner_state.transitions[desired_path]
    local tp_available = type(available)
    if tp_available == 'string' then
      next_path = available
    elseif tp_available=='table' then
      -- Just take the first :P
      local ind_rand = math.random(#available)
      next_path = available[ind_rand]
    end
  end

  -- New path implies a new controller
  if pp_params.path ~= my_path or not pp_control then
    pp_params.path = my_path
    -- Highway points are different
    if my_path.markers then
      pp_control, pp_err = control.simple_pursuit(pp_params)
    else
      -- KD Tree of points
      my_path:generate_kdtree()
      -- If we can see the next path, use it
      pp_params.path_next = next_path
      -- Form our new controller
      pp_control, pp_err = control.pure_pursuit(pp_params)
    end
  end

  -- Check if there is a controller
  if not pp_control then return false, pp_err end

  local lane_desired
  -- Find our control policy
  if my_path.markers then
    -- The points are the lanes. All angles are 0
    local lane_to_lane_dist = tonumber(my_path.lane_width) or 0.5
    -- Make speed dependent, like 3 seconds ahead at 1 m/s
    -- 3 meters of highway lookahead.
    local seconds_lookahead = 3
    local px_lookahead = pose_rbt[1] + seconds_lookahead * pp_params.velocity

    -- TODO: Make nicer...
    if houston_event == 'left' then
      lane_desired = 1
    elseif houston_event == 'right' then
      lane_desired = -1
    else
      lane_desired = 0
    end
    -- Add 0.5 to go halfway within the lane
    local p_lookahead = {px_lookahead, (lane_desired + 0.5) * lane_to_lane_dist, 0}
    pp_result, pp_err = pp_control(pose_rbt, p_lookahead)
  else
    pp_result, pp_err = pp_control(pose_rbt)
  end

  -- Check for errors
  if not pp_result then
    return false, pp_err
  end
  if pp_result.err then
    pp_err = pp_result.err
    return false, pp_err
  end

  -- Check if we are done, and then set the next path
  if pp_result.done then
    desired_path = next_path
    next_path = false
  end

  -- Ensure we add our ID to the result
  pp_result.id = id_robot
  pp_result.pathname = desired_path
  -- Save the pose, just because
  pp_result.pose = pose_rbt

  --------------------------------
  -- Check the obstacles around us, if not a highway, for now
  -- Assign all vehicles to a lane
  local veh_lanes = {}
  for name_veh, pose_veh in pairs(veh_poses) do
    -- Back axle
    veh_lanes[name_veh] = pp_params.path:find(pose_veh, {closeness=0.25})
    -- Front left
    -- pose_veh + vector.pose{}
    -- veh_lanes[name_veh] = pp_params.path:find(pose_veh)
    -- Front right
    -- veh_lanes[name_veh] = pp_params.path:find(pose_veh)
  end
  -- Find the lead vehicle
  local d_lead = math.huge
  local name_lead = false
  if my_path.markers then
    local lane_current = veh_lanes[id_robot].i_lane
    local lane_keep = lane_current == lane_desired
    for name_veh, info_lane in pairs(veh_lanes) do
      local veh_pose = veh_poses[name_veh]
      local dx = veh_pose[1] - pose_rbt[1]
      -- local rel_pose
      if info_lane.i_lane == name_veh then

      elseif lane_keep and info_lane.i_lane == lane_current and dx > 0 then
        if dx < d_lead then
          name_lead = name_veh
          d_lead = dx
        end
      elseif (not lane_keep) and info_lane.i_lane == lane_desired and dx > 0 then
        if dx < d_lead then
          name_lead = name_veh
          d_lead = dx
        end
      end
    end
  else
    local name, dx = find_lead(veh_lanes, pp_params.path, pp_result.id_path)
    -- Slow for a lead vehicle
    if not name then
      -- Save the message
      name_lead = dx
    elseif dx < d_lead then
      d_lead = dx
    end
  end
  pp_params.leader = {name_lead, d_lead}
  -- Bound the ratio
  local ratio = 1
  -- Based on the measurements: rear axle to other car rear bumper
  local d_stop = wheel_base + 0.22
  local d_near = 3 * wheel_base
  -- print("d_lead", d_lead)
  if d_lead < d_near then
    ratio = (d_lead - d_stop) / (d_near - d_stop)
    ratio = math.max(0, math.min(ratio, 1))
  end
  --------------------------------

  -- When looping, update the velocity
  local vel_sample = pp_params.velocity
  -- Just update all the time, now
  if true or pp_result.id_path == 1 then
    local sample = vector.randn(1, pp_params.velocity_stddev, pp_params.velocity)
    -- Bound the sample
    vel_sample = math.max(pp_params.velocity_min, math.min(sample[1], pp_params.velocity_max))
  end

  if pp_result.done then
    pp_result.steering = 0
    pp_result.velocity = 0
  else
    -- Set the steering based on the car dimensions
    local steering = atan(pp_result.kappa * wheel_base)
    pp_result.steering = steering
    -- TODO: Set the speed based on curvature? Lookahead point's curvature?
    pp_result.velocity = vel_sample * ratio
  end

  -- Broadcast
  log_announce(log, pp_result, "control")
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

local function cb_plan(msg, ch, t_us)
  -- Update the information available
  -- print("Got plan from", ch, "at", t_us)
  planner_state = msg
end

local function cb_houston(msg, ch, t_us)
  houston_event = id_robot=='tri1' and msg.evt
end

local cb_tbl = {
  vicon = parse_vicon,
  planner = cb_plan,
  houston = cb_houston
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
