#!/usr/bin/env luajit
local coresume = require'coroutine'.resume
local cocreate = require'coroutine'.create
local max, min = require'math'.max, require'math'.min
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local desired_path = flags.desired or 'lane_outer'
local my_id = assert(racecar.HOSTNAME)

local control = require'control'
local kdtree = require'kdtree'
local has_logger, logger = pcall(require, 'logger')
local vector = require'vector'
local log_announce = racecar.log_announce
local log = has_logger and flags.log~=0
            and assert(logger.new('control', racecar.HOME.."/logs"))

local entered_intersection, min_lane_dist, obs_lane_dist
local lookahead = 0.6
local wheel_base = 0.3
local ok_to_go = true
local ignore_risk = false
local risk_nogo = 0.03
local vel_h = false
local vel_max = 0.75
local vel_l = 0.5

local cofsm = require'cofsm'
local fsm_control = cofsm.new{
  {'botStop', 'go', 'botGo'},
  {'botGo', 'stop', 'botStop'}
}
print('fsm_control', fsm_control.current_state)

local waypoints = {}
waypoints.lane_inner = {
  {1.5, 1}, {1.5, -0.75},
  {1, -1.25}, {0, -1.25},
  {-0.5, -0.75}, {-0.5, 3.5},
  {0, 4}, {1, 4},
  {1.5, 3.5}, {1.5, 1.5}
}
waypoints.lane_outer =  {
  {2.1, 1.5}, {2.1, 4.25},
  {1.6, 4.75}, {-0.75, 4.75},
  {-1.25, 4.25}, {-1.25, -1.5},
  {-0.75, -2}, {1.6, -2},
  {2.1, -1.5}, {2.1, 1}
}
waypoints.lane_enter = {
  {-2.5, 1.5}, {-1.25, 1.5}
}
waypoints.turn_left = {
  {-1.5, 1.5}, {-0.8, 1.5}, {-0.7705948579011315, 1.5014445819983409}, {-0.7414729033951611, 1.5057644158790309}, {-0.712914596823661, 1.5129178992803374}, {-0.6851949702904727, 1.5228361402466142}, {-0.6585809789522005, 1.5354236206954937}, {-0.6333289300941191, 1.5505591163092367}, {-0.6096820147509061, 1.5680968639911792}, {-0.5878679656440355, 1.587867965644036}, {-0.5680968639911788, 1.6096820147509066}, {-0.5505591163092364, 1.6333289300941196}, {-0.5354236206954934, 1.6585809789522008}, {-0.522836140246614, 1.6851949702904732}, {-0.5129178992803374, 1.7129145968236614}, {-0.5057644158790309, 1.7414729033951617}, {-0.5014445819983409, 1.770594857901132}, {-0.5, 1.8},
}
waypoints.turn_right = {
  {-1.5, 1.5}, {-1.5205948579011328, 1.4985554180016591}, {-1.4914729033951626, 1.4942355841209694}, {-1.4629145968236625, 1.4870821007196628}, {-1.4351949702904743, 1.4771638597533865}, {-1.408580978952202, 1.464576379304507}, {-1.3833289300941205, 1.4494408836907644}, {-1.3596820147509077, 1.4319031360088221}, {-1.3378679656440369, 1.4121320343559653}, {-1.31809686399118, 1.390317985249095}, {-1.3005591163092374, 1.3666710699058822}, {-1.2854236206954943, 1.341419021047801}, {-1.2728361402466148, 1.3148050297095288}, {-1.262917899280338, 1.2870854031763408}, {-1.2557644158790313, 1.2585270966048407}, {-1.251444581998341, 1.2294051420988705}, {-1.25, 1.2},
}

local paths = {}
local ds = 0.05
for k, wps in pairs(waypoints) do
  local path, length = control.gen_path(wps, ds)
  local tree = kdtree.create(2)
  for i, p in ipairs(path) do tree:insert(p, i) end
  paths[k] = path
  paths[k].tree = tree
  paths[k].length = length
end

----------------------
local my_path
local straight_start = false
if desired_path:find'turn' and paths[desired_path] then
  straight_start = true
  my_path = paths.lane_enter
else
  my_path = assert(paths[desired_path], "No desired path found: "..tostring(desired_path))
end
-- local env = {
--   viewBox = {-3, -5.5, 7, 9},
--   observer = vector.pose(),
--   time_interval = 0.1,
--   speed = 0.1,
--   lanes = {waypoints.lane_inner, waypoints.lane_outer},
--   trajectory_turn = {waypoints.traj_left_turn, waypoints.traj_right_turn},
-- }

-- meters
local threshold_close = tonumber(flags.threshold_close) or 1

-- Give the position, path id, distance to the point
local function fn_nearby(id_last, p_lookahead)
  if not p_lookahead then
    local id_nearby = id_last + 1
    return id_nearby, my_path[id_nearby]
  end
  local nearby = my_path.tree:nearest(p_lookahead, threshold_close)
  -- NOTE: nearby should be sorted by increasing distance
  if not nearby then
    return false, "No points nearby the path"
  end
  if not id_last then
    return nearby[1].user, math.sqrt(nearby[1].dist_sq)
  end
  local id_nearby, dist_nearby
  for _, nby in ipairs(nearby) do
    if nby.user >= id_last then
      id_nearby = nby.user
      dist_nearby = nby.dist_sq
      break
    end
  end
  if not id_nearby then
    return false , "No unvisited points"
  end
  -- Don't skip too far in a single timestep
  -- if id_nearby > id_last + 1 then
  --   id_nearby = id_last + 1
  -- end
  return id_nearby, dist_nearby
end

local co_control = cocreate(control.pure_pursuit{
                            path=my_path,
                            fn_nearby=fn_nearby,
                            lookahead=lookahead})

-- Given car pose
local function find_lane(p_vehicle)
  local kmin, dmin, imin = false, math.huge, false
  for k, path in pairs(paths) do
    if k:match"^lane" then
      local nearest, err = path.tree:nearest({unpack(p_vehicle, 1, 2)}, 1)
      nearest = nearest and nearest[1]
      -- TODO: Check dot product direction
      if nearest then
        --print("Near", k, nearest.dist_sq)
        if nearest.dist_sq < dmin then
          imin = nearest.user
          dmin = nearest.dist_sq
          kmin = k
        end
      --else
        --print("Not near!", err, path.tree:size(), k, p_vehicle)
      end
    end
  end
  return {name_path = kmin, id_path=imin, dist_sq=dmin}
end

local function vicon2pose(vp)
  return vp.translation[1] / 1e3, vp.translation[2] / 1e3, vp.rotation[3]
end

local last_frame = -math.huge
local function parse_vicon(msg)
  -- TODO: Stale for each ID...
  -- may be the latest for that vehicle
  local frame = msg.frame
  if frame < last_frame then
    return false, "Stale data"
  end
  last_frame = frame
  local poses, lanes = {}, {}
  for id, vp in pairs(msg) do
    if id~='frame' then
      local p = vector.pose{vicon2pose(vp)}
      poses[id] = p
      lanes[id] = find_lane(p)
    end
  end
  -- Update the robot pose
  local pose_rbt = poses[my_id]
  if not pose_rbt then return end
  -- Check if a car is in my lane :)
  local my_lane = lanes[my_id]
  lanes[my_id] = nil

  -- If in a lane, then see who is our lead vehicle
  local lead_offset, lead_vehicle = math.huge, nil
  if my_lane then
    for id, lane in pairs(lanes) do
      if lane.name_path==my_lane.name_path then
        --print(lane.id_path, my_lane.id_path, lane.name_path, my_lane.name_path)
        -- TODO: Check the relative pose between us and that ID
        local path_offset = (lane.id_path - my_lane.id_path) * ds
        --print(id, "in my lane", lane.id_path, "distance", path_offset)
        if path_offset > 0 then
          if path_offset < lead_offset then
            lead_vehicle = id
            lead_offset = path_offset
          end
        end
  --    else
  --      print(id, "not in my lane", lane.name_path)
      end
    end
  end

  local running, result, err = coresume(co_control, pose_rbt)
  if not running then
    print("Not running", result)
    log_announce(log, { steering = 0, rpm = 0 }, "control")
    return os.exit()
  elseif type(result)~='table' then
    print("Improper", result, err)
    log_announce(log, { steering = 0, rpm = 0 }, "control")
    return
  elseif result.err then
    print("Error", result.err)
    log_announce(log, { steering = 0, rpm = 0 }, "control")
    return
  elseif result.done then
    print("DONE!", desired_path)
    if straight_start then
      straight_start = false
      my_path = paths[desired_path]
    elseif desired_path:find"left" then
      ignore_risk = true
      desired_path = 'lane_inner'
    elseif desired_path:find"right" then
      ignore_risk = true
      desired_path = 'lane_outer'
    end
    my_path = paths[desired_path]
    -- Keep looping
    co_control = cocreate(control.pure_pursuit{
                          path=my_path,
                          fn_nearby=fn_nearby,
                          lookahead=lookahead,
                          id_start=1
                          })
    return
  end

  local steering = math.atan(result.kappa * wheel_base)

  -- For sending to the vesc
  result.steering = steering
  -- result.duty = 6

  local vel_v = vel_h or vel_l

  if fsm_control.current_state == 'botStop' then
    -- print("stopped")
    -- result.duty = 0
    vel_v = 0
  elseif desired_path=='lane_outer' or desired_path=='lane_inner' then
    print("in path")
    local d_stop = 0.8
    local d_near = 1.5
    local ratio = (lead_offset - d_stop) / (d_near - d_stop)
    ratio = max(0, min(ratio, 1))
    vel_v = ratio * vel_v
    -- if lead_offset < d_near then
    --   print(string.format("Stopping for %s | [%.2f -> %.2f]",
    --                       lead_vehicle, ratio, result.rpm or result.duty))
    -- end
  elseif entered_intersection==false or straight_start then
    if math.abs(vel_v) >= 0.25 then
      local ratio = math.abs(d_j or 1.6) / 1.6
      vel_v = vel_v * max(0, min(ratio, 1))
      if vel_v < 0.25 and vel_v>=0 then
        vel_v = 0.25 
      elseif vel_v > -0.25 and vel_v <= 0 then
        vel_v = -0.25
      end
    end
    if not ok_to_go and math.abs(d_j) < 0.05 then vel_v = math.min(0, vel_v) end
    print("Not entered", d_j, ok_to_go, vel_v)
    print("vel_v", vel_v)
  elseif entered_intersection then
    -- TODO: Check t_clear
    if max_t_clear then
      min_vel_clear = paths.turn_left.length / max_t_clear
      vel_v = math.max(vel_v, min_vel_clear)
    end
    print("min_vel_clear", min_vel_clear)
    print("vel_v", vel_v)
  end
  -- print('result.rpm', result.rpm)
  vel_v = math.max(-vel_max, math.min(vel_v, vel_max))
  result.rpm = vel_v * racecar.RPM_PER_MPS

  -- Keep track of our state
  result.current_state = fsm_control.current_state

  log_announce(log, result, "control")
  -- For GUI plotting
  -- env.observer = pose_rbt
  -- log_announce(log, env, "risk")
end

local function parse_risk(msg)
  if msg.go ~= ok_to_go then
    ok_to_go = msg.go
    if ignore_risk then
      ok_to_go = true
    elseif desired_path~='lane_outer' and desired_path~='lane_inner' then
      print("OK to go?", ok_to_go)
    end
  end
  max_t_clear = msg.max_t_clear
  d_j = msg.d_j
  gap = msg.gap
  entered_intersection = msg.d_j > 0
end

local function parse_houston(msg)
  print("Received a command from Houston!")
  for k, v in pairs(msg) do
    print(k, v)
  end
  -- Dispatch events
  local ret, err = fsm_control:dispatch(msg.evt)
  if not ret then print("FSM", err) end
end

-- local JOYSTICK_TO_DUTY = 11 / 32767
-- local JOYSTICK_TO_RPM = 30000 / 32767
local function parse_joystick(msg)
  if type(msg.buttons)~='table' or type(msg.axes)~='table' then
    return
  end
  vel_h = vel_max * msg.axes[2] / -32767
end

local cb_tbl = {
  vicon = parse_vicon,
  risk = parse_risk,
  houston = parse_houston,
  joystick = parse_joystick,
}

racecar.listen{
  channel_callbacks = cb_tbl,
}

print("Done control!")
