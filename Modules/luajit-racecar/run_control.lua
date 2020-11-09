#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local id_robot = flags.id or racecar.HOSTNAME
assert(id_robot, "No name provided!")
math.randomseed(123)
local atan = require'math'.atan
local fabs = require'math'.abs
local vector = require'vector'

local control = require'control'
local highway = require'highway'
local path = require'path'
local tf2D = require'transform'.tf2D

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Initialize the system
racecar.init()

-- Globally accessible variables
local path_list = {} -- Path list now and into the future. Should hold path_rollout_time seconds
local path_rollout_time = 5 -- How far ahead to look given the speed limit of each path
local planner_state = false
-- Parameters for each robot
local vehicle_params = {}

local function vehicle_new(params)
  if type(params) ~= 'table' then params = {} end
  local velocity_mean = tonumber(params.vel_lane) or 0.5
  local velocity_stddev = velocity_mean * 0.1
  local vel_max = 0.75
  local vel_min = 0.2
  local nan = 0/0
  local wheel_base = tonumber(params.wheel_base) or 0.325
  -- TODO: Should simply find the nearest lane
  local pp_params = {
    id = params.id,
    pose = {nan, nan, nan}, -- the pose that we think that we have
    --
    wheel_base = wheel_base,
    t_lookahead = tonumber(params.lookahead) or 1,
    velocity_mean = velocity_mean,
    velocity_stddev = velocity_stddev,
    velocity_max = vel_max,
    velocity_min = vel_min,
    limiting_radius = 2 * wheel_base,
    --
    pathname = params.path or 'lane_outerA+1',
    lane_desired = false,
    lane_current = false,
    leader = {false, math.huge},
    --
    velocity = 0,
    steering = 0,
  }
  return pp_params
end
vehicle_params[id_robot] = vehicle_new(flags)

--------------------------
-- Update the pure pursuit
-- This function modifies the parameters table in place
local function update_params(pp_params)
  if type(pp_params) ~= 'table' then
    return false, "No parameters given"
  end

  -- The controller, by default, stops
  pp_params.steering = 0
  pp_params.velocity = 0

  local pose_rbt = pp_params.pose
  if not pose_rbt then
    -- Ensure that we have a pose
    return false, "No pose"
  elseif not pp_params.pathname then
    -- Check that we can find a plan
    return false, "No pathname"
  end

  -- Find our plan in paths and highways
  local my_path = planner_state.paths and planner_state.paths[pp_params.pathname]
  local my_highway = planner_state.highways and planner_state.highways[pp_params.pathname]
  if my_path then
    my_path = path.wrap(my_path)
    my_path:generate_kdtree()
  elseif my_highway then
    my_path = highway.wrap(my_highway)
  else
    return false, "Desired path not found in the planner"
  end

  -- Find the next path
  if planner_state.transitions and not pp_params.path_next then
    local available = planner_state.transitions[pp_params.pathname]
    local tp_available = type(available)
    if tp_available == 'string' then
      pp_params.path_next = available
    elseif tp_available=='table' then
      local ind_rand = math.random(#available)
      pp_params.path_next = available[ind_rand]
    end
  end

  -- Assign all vehicles to a lane
  local closeness = 0.4
  for _, params_veh in pairs(vehicle_params) do
    local pose_veh = params_veh.pose
    -- Back axle
    local path_info, err_find = my_path:find(pose_veh, {
      closeness=closeness,
      orientation_threshold=math.rad(60)
    })
    params_veh.lane_current = path_info and path_info.idx_lane or err_find
    params_veh.longitudinal_id_current = path_info and path_info.idx_path or err_find
    if not params_veh.lane_desired then
      params_veh.lane_desired = params_veh.lane_current
    end
    -- Front left
    local dx_bumper, dy_bumper = 0.10, 0.05
    local x_bumper, y_bumper = tf2D(dx_bumper, dy_bumper, pose_veh[3], unpack(pose_veh, 1, 2))
    -- veh_lanes[name_veh] = pp_params.path:find({x_bumper, y_bumper, pose_veh[3]})
    -- Front right
    dy_bumper = -1 * dy_bumper
    x_bumper, y_bumper = tf2D(dx_bumper, dy_bumper, pose_veh[3], unpack(pose_veh, 1, 2))
    -- veh_lanes[name_veh] = pp_params.path:find({x_bumper, y_bumper, pose_veh[3]})
  end

  --------------------------------
  -- Check the obstacles around us, if not a highway, for now
  if type(pp_params.longitudinal_id_current) ~= 'number' then
    local path_info, err_find = my_path:find(pose_rbt)
    if not path_info then
      pp_params.lane_current = err_find
      pp_params.longitudinal_id_current = err_find
      return false, "No pose on path: "..tostring(err_find)
    end
    pp_params.lane_current = path_info.idx_lane
    pp_params.longitudinal_id_current = path_info.idx_path
  end

  -- Find the lead vehicle
  -- TODO: We want to find the time to the leader, maybe not the Distance
  -- NOTE: Currently, this assumes a stopped vehicle
  local d_lead = math.huge
  local name_lead = false
  for name_veh, params_veh in pairs(vehicle_params) do
    local is_other = name_veh ~= pp_params.id
    local dx
    if my_path.markers then
      -- highway
      local pose_veh = params_veh.pose
      dx = pose_veh[1] - pose_rbt[1]
    else
      -- In curvy area, use the path distance
      local d_id = my_path:relative_id_diff(pp_params.longitudinal_id_current, params_veh.longitudinal_id_current)
      dx = my_path.ds * d_id
    end
    local veh_in_current_lane = params_veh.lane_current == pp_params.lane_current
    local veh_in_desired_lane = params_veh.lane_current == pp_params.lane_desired
    -- Skip our vehicle and any behind us
    -- Skip if further away than the lead vehicle
    if is_other and dx < d_lead and (dx + params_veh.wheel_base) > 0 and (veh_in_desired_lane or veh_in_current_lane) then
      name_lead = name_veh
      d_lead = dx
    end
  end
  pp_params.leader = {name_lead, d_lead}
  -- Based on the measurements: rear axle to other car rear bumper
  local d_stop = pp_params.wheel_base + 0.22
  local d_near = 3 * pp_params.wheel_base
  local vel_ratio_lead = (d_lead - d_stop) / (d_near - d_stop)
  --------------------------------

  -- Find the curvature
  local vel_desired = vector.randn(1, pp_params.velocity_stddev, pp_params.velocity_mean)[1]
  local d_lookahead = pp_params.t_lookahead * vel_desired
  -- Find our control policy
  if my_path.markers then
    -- The points are the lanes. All angles are 0
    local lane_to_lane_dist = tonumber(my_path.lane_width) or 0.5
    -- Make speed dependent, like 3 seconds ahead at 1 m/s
    local px_lookahead = pose_rbt[1] + d_lookahead
    -- Find the highway centerline
    local py_path = (pp_params.lane_current + 0.5) * lane_to_lane_dist
    local py_lookahead = (pp_params.lane_desired + 0.5) * lane_to_lane_dist
    -- Add 0.5 to go halfway within the lane
    pp_params.p_lookahead = {px_lookahead, py_lookahead, 0}
    pp_params.p_path = {pose_rbt[1], py_path, 0}
  else
    -- Distance to path
    local p_path = my_path.points[pp_params.longitudinal_id_current]
    pp_params.p_path = p_path
    local dx, dy = pose_rbt[1] - p_path[1], pose_rbt[2] - p_path[2]
    pp_params.d_path = math.sqrt(dx * dx + dy * dy)
    -- Prepare the lookahead point
    local p_lookahead = {tf2D(d_lookahead, 0, pose_rbt[3], pose_rbt[1], pose_rbt[2])}
    -- Find the nearest path point to the lookahead point
    -- TODO: Don't skip too far in a single timestep?
    local function forwards(id)
      return (my_path:relative_id_diff(pp_params.longitudinal_id_current, id) > 0) and id, true
    end
    local id_path_lookahead = my_path:nearby(p_lookahead, d_lookahead, forwards)
    -- Default to the lookahead from our point
    if not id_path_lookahead then
      id_path_lookahead = my_path:get_id_ahead(pp_params.longitudinal_id_current, d_lookahead)
    end
    pp_params.id_path_lookahead = id_path_lookahead
    pp_params.p_lookahead = my_path.points[id_path_lookahead]
  end
  local pp_result = control.get_inverse_curvature(pose_rbt, pp_params.p_lookahead)
  pp_params.alpha = pp_result.alpha
  pp_params.kappa = pp_result.kappa
  pp_params.radius_of_curvature = pp_result.radius_of_curvature
  local steering_desired = atan(pp_params.kappa * pp_params.wheel_base)

  -- Update the velocity, based on the turning radius
  local vel_ratio_turning = math.abs(pp_params.radius_of_curvature) / pp_params.limiting_radius

  -- Find out if we are done, based on the path id of the lookahead point and our current point
  if my_path.markers then
  elseif pp_params.path_next then
    if pp_params.id_path_lookahead==#my_path.points then
      pp_params.pathname = pp_params.path_next
      pp_params.path_next = false
    end
  elseif my_path.closed then
  elseif pp_params.id_path_pose==#my_path.points then
    -- No next pose
    pp_params.done = true
  end

  -- Check if we are done
  if pp_params.done then
    pp_params.steering = 0
    pp_params.velocity = 0
  else
    -- Set the steering based on the car dimensions
    pp_params.steering = steering_desired
    -- Bound the sample
    local vel_ratio = math.max(0, math.min(1, vel_ratio_turning, vel_ratio_lead))
    vel_desired = vel_ratio * vel_desired
    if vel_desired < pp_params.velocity_min then
      pp_params.velocity = 0
    else
      pp_params.velocity = math.min(vel_desired, pp_params.velocity_max)
    end
  end
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
local function cb_vicon(msg)
  -- Check that the data is not stale
  -- TODO: Stale for each ID...
  local frame = msg.frame
  if frame <= last_frame then
    return false, "Stale data"
  end
  last_frame = frame
  -- Find the pose for each robot
  msg.frame = nil
  for id, vp in pairs(msg) do
    local pp_params = vehicle_params[id]
    if not pp_params then
      pp_params = vehicle_new{
        id = id,
        path = vehicle_params[id_robot].pathname
      }
      vehicle_params[id] = pp_params
    end
    pp_params.pose = vicon2pose(vp)
  end
end
-- Update the poses
-------------------

local function cb_plan(msg, ch, t_us)
  -- Update the information available
  planner_state = msg
end

local function cb_houston(msg, ch, t_us)
  local houston_id = msg.id
  local houston_event = msg.evt
  local houston_value = msg.val
  if not houston_event or not houston_id then
    return false
  end
  for name_veh, params_veh in pairs(vehicle_params) do
    if name_veh:match(houston_id) then
      if houston_event == 'lane' and type(houston_value)=='number' then
        params_veh.lane_desired = math.floor(houston_value)
      elseif houston_event == 'lane' and type(houston_value)=='string' then
        if houston_value=='left' then
          params_veh.lane_desired = params_veh.lane_desired + 1
        elseif houston_value=='right' then
          params_veh.lane_desired = params_veh.lane_desired - 1
        end
      elseif houston_event == 'velocity' and type(houston_value)=='number' then
        params_veh.velocity_mean = houston_value
      elseif houston_event == 'parameters' and type(houston_value)=='table' then
        -- Shallow copy
        for k, v in pairs(houston_value) do params_veh[k] = v end
      end
    end
  end
end

local function cb_loop(t_us)
  if not planner_state then
    return false, "No planner information"
  end

  -- Update the control parameters
  for _, params_veh in pairs(vehicle_params) do
    update_params(params_veh)
    -- Broadcast all!
    log_announce(log, params_veh, "control")
  end
  -- Broadcast just ours
  -- log_announce(log, vehicle_params[id_robot], "control")
end

local function cb_debug(t_us)
  local info = {"=="}
  for name_veh, params_veh in pairs(vehicle_params) do
    table.insert(info, string.format(
      "\n%s | Pose: x=%.2fm, y=%.2fm, a=%.2f°",
      name_veh, params_veh.pose[1], params_veh.pose[2], math.deg(params_veh.pose[3])))
    table.insert(info, string.format(
      "Path: %s | Lane: %s | Next: %s",
      params_veh.pathname, params_veh.lane_current, params_veh.path_next))
    table.insert(info, string.format(
      "Leader: %s [%s]",
      unpack(params_veh.leader)))
    table.insert(info, string.format(
      "Control: %.2f m/s, %.2f°",
      params_veh.velocity, math.deg(params_veh.steering)))
  end
  return table.concat(info, "\n")
end

local function exit()
  -- Stop all the vehicles
  for _, pp_params in pairs(vehicle_params) do
    pp_params.steering = 0
    pp_params.velocity = 0
    log_announce(log, pp_params, "control")
  end
  if log then log:close() end
  return 0
end
-- racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = {
    vicon = cb_vicon,
    planner = cb_plan,
    houston = cb_houston
  },
  loop_rate = 100, -- 100ms loop
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
