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
local houston_event = false
local planner_state = false
-- Parameters for each robot
local vehicle_params = {}

local function vehicle_new(params)
  if type(params) ~= 'table' then params = {} end
  local velocity_mean = tonumber(params.vel_lane) or 0.5
  local velocity_stddev = velocity_mean * 0.1
  local vel_max = 0.75
  local vel_min = 0.2
  -- TODO: Should simply find the nearest lane
  local pp_params = {
    id = id_robot,
    pose = false, -- the pose that we think that we have
    --
    wheel_base = tonumber(params.wheel_base) or 0.325,
    t_lookahead = tonumber(params.lookahead) or 1,
    threshold_close = 0.25,
    velocity_mean = velocity_mean,
    velocity_stddev = velocity_stddev,
    velocity_max = vel_max,
    velocity_min = vel_min,
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

--
local function cb_debug(t_us)
  local pp_params = vehicle_params[id_robot]
  local pose_rbt = pp_params.pose
  if not pose_rbt then return end
  local px, py, pa = unpack(pose_rbt)
  local info = {
    string.format("Path: %s | Next: %s", pp_params.pathname, pp_params.path_next),
  }
  table.insert(info, string.format(
    "Pose: x=%.2fm, y=%.2fm, a=%.2f°",
    px, py, math.deg(pa)))
  table.insert(info, string.format(
    "Leader: %s [%s]",
    unpack(pp_params.leader)))
  table.insert(info, string.format(
    "Control: %.2f m/s, %.2f°",
    pp_params.velocity, math.deg(pp_params.steering)))
  return table.concat(info, "\n")
end

------------------------------------------
-- Utilities
------------------------------------------

local function find_lead(self, veh_lanes, id_path)
  -- Cache
  local is_closed = self.closed
  local n_points = #self.points
  local ds = self.ds

  local name_lead, d_lead = false, math.huge
  for name_veh, info_veh in ipairs(veh_lanes) do
    local id_path_other = info_veh.id_in_path
    local d_id = id_path_other - id_path
    -- Loop around if closed
    if is_closed then
      if id_path_other < id_path then
        id_path_other = id_path_other + n_points
      else
        id_path_other = id_path_other - n_points
      end
      local d_id2 = id_path_other - id_path
      if fabs(d_id2) < fabs(d_id) then
        d_id = d_id2
      end
    end
    local d_rel = d_id * ds
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

--------------------------
-- Update the pure pursuit
local function update_params(pp_params)
  local pose_rbt = pp_params.pose
  -- Stop if no pose
  if not pose_rbt then
    pp_params.steering = 0
    pp_params.velocity = 0
    return false, "No pose"
  end

  -- Check that we can find a plan
  if not pp_params.pathname then
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

  if my_path.markers then
    if houston_event == 'left' then
      pp_params.lane_desired = 1
    elseif houston_event == 'right' then
      pp_params.lane_desired = -1
    else
      pp_params.lane_desired = 0
    end
  else
    -- Find the next path
    if planner_state.transitions and not pp_params.path_next then
      local available = planner_state.transitions[pp_params.pathname]
      local tp_available = type(available)
      if tp_available == 'string' then
        pp_params.path_next = available
      elseif tp_available=='table' then
        -- Just take the first :P
        local ind_rand = math.random(#available)
        pp_params.path_next = available[ind_rand]
      end
    end
  end
  local vel_sample = vector.randn(1, pp_params.velocity_stddev, pp_params.velocity_mean)[1]

  --------------------------------
  -- Check the obstacles around us, if not a highway, for now
  -- Assign all vehicles to a lane
  local veh_lanes = {}
  for _, params_veh in pairs(vehicle_params) do
    local pose_veh = params_veh.pose
    -- Back axle
    local path_info = my_path:find(pose_veh, {closeness=0.25})
    params_veh.lane_current = path_info and path_info.idx_lane
    -- Front left
    local dx_bumper, dy_bumper = 0.10, 0.05
    local x_bumper, y_bumper = tf2D(dx_bumper, dy_bumper, pose_veh[3], unpack(pose_veh, 1, 2))
    -- veh_lanes[name_veh] = pp_params.path:find({x_bumper, y_bumper, pose_veh[3]})
    -- Front right
    dy_bumper = -1 * dy_bumper
    x_bumper, y_bumper = tf2D(dx_bumper, dy_bumper, pose_veh[3], unpack(pose_veh, 1, 2))
    -- veh_lanes[name_veh] = pp_params.path:find({x_bumper, y_bumper, pose_veh[3]})
  end
  -- Find the lead vehicle
  local d_lead = math.huge
  local name_lead = false
  if my_path.markers then
    -- local lane_current = veh_lanes[id_robot].i_lane
    -- local lane_keep = lane_current == pp_params.lane_desired
    for name_veh, params_veh in pairs(vehicle_params) do
      local is_other = name_veh ~= pp_params.id
      local pose_veh = params_veh.pose
      local dx = pose_veh[1] - pose_rbt[1]
      -- local veh_in_current_lane = params_veh.idx_lane == lane_current
      local veh_in_desired_lane = params_veh.lane_current == pp_params.lane_desired
      -- Skip our vehicle and any behind us
      -- Skip if further away than the lead vehicle
      if is_other and veh_in_desired_lane and (dx + params_veh.wheel_base) > 0 and dx < d_lead then
        name_lead = name_veh
        d_lead = dx
      end
    end
  else
    local name, dx = find_lead(my_path, veh_lanes, pp_params.id_path)
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
  local d_stop = pp_params.wheel_base + 0.22
  local d_near = 3 * pp_params.wheel_base
  if d_lead < d_near then
    ratio = (d_lead - d_stop) / (d_near - d_stop)
    ratio = math.max(0, math.min(ratio, 1))
  end
  --------------------------------

  -- Find the curvature
  local p_lookahead
  local d_lookahead = pp_params.t_lookahead * pp_params.velocity
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
    p_lookahead = {px_lookahead, py_lookahead, 0}
    pp_params.p_path = {pose_rbt[1], py_path, 0}
  else
    -- Find ourselves on the path
    -- TODO: Threshold and find on the closest road
    local id_path, err = my_path:nearby(pose_rbt)
    pp_params.id_path_pose = id_path
    if not id_path then
      return false, "No pose on path: "..tostring(err)
    end
    -- Distance to path
    local p_path = my_path.points[id_path]
    pp_params.p_path = p_path
    local dx, dy = pose_rbt[1] - p_path[1], pose_rbt[2] - p_path[2]
    pp_params.d_path = math.sqrt(dx * dx + dy * dy)
    -- Prepare the lookahead point
    p_lookahead = {tf2D(d_lookahead, 0, pose_rbt[3], pose_rbt[1], pose_rbt[2])}
    -- Find the nearest path point to the lookahead point
    -- TODO: Don't skip too far in a single timestep?
    local function forwards(id) return (id > id_path) and id, true end
    local id_path_lookahead = my_path:nearby(p_lookahead, d_lookahead, forwards)
    -- Default to the lookahead from our point
    if not id_path_lookahead then
      id_path_lookahead = my_path:get_id_ahead(id_path, d_lookahead)
    end
    pp_params.id_path_lookahead = id_path_lookahead
    p_lookahead = my_path.points[id_path_lookahead]
  end
  pp_params.p_lookahead = p_lookahead
  local pp_result = control.get_inverse_curvature(pose_rbt, p_lookahead)
  pp_params.alpha = pp_result.alpha
  pp_params.kappa = pp_result.kappa
  pp_params.radius_of_curvature = pp_result.radius_of_curvature
  local steering_sample = atan(pp_params.kappa * pp_params.wheel_base)

  local LIMITING_RADIUS = 1.5
  -- print("Radius", pp_params.radius_of_curvature)
  ratio = math.min(math.abs(pp_params.radius_of_curvature) / LIMITING_RADIUS, ratio)

  -- Update the velocity, based on the lead vehicle
  vel_sample = vel_sample * ratio

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
    pp_params.steering = steering_sample
    -- Bound the sample
    pp_params.velocity = math.max(pp_params.velocity_min, math.min(vel_sample, pp_params.velocity_max))
  end
end
-- Update the pure pursuit
--------------------------

local function cb_loop(t_us)
  if not planner_state then
    return false, "No planner information"
  end
  for _, params_veh in pairs(vehicle_params) do
    update_params(params_veh)
  end
  -- Broadcast just ours, or all of them?
  log_announce(log, vehicle_params[id_robot], "control")
end

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
      print("New car!")
      pp_params = vehicle_new()
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
  houston_event = id_robot=='tri1' and msg.evt
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
racecar.handle_shutdown(exit)

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
