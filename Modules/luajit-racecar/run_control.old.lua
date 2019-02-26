#!/usr/bin/env luajit
local coresume = require'coroutine'.resume
local cocreate = require'coroutine'.create
local max, min = require'math'.max, require'math'.min
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local desired_turn = flags.turn or 'turn_left'
local id_rbt = assert(racecar.HOSTNAME)

local control = require'control'
local kdtree = require'kdtree'
local has_logger, logger = pcall(require, 'logger')
local vector = require'vector'
local log_announce = racecar.log_announce
local log = has_logger and flags.log~=0
            and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Globally accessible variables
local lookahead = 0.6
local wheel_base = 0.3
local vel_h = false
local vel_max = 0.75 -- 1 --0.75
local vel_min = 0.2
local vel_l = 0.5 -- 0.75 --0.5
local my_path
local co_control
local my_path_name

-- Use tables, since easier
local poses = {}
local lanes = {}
local risk = {}

local waypoints = {}

waypoints.lane_inner = {
  {3.5, 2}, {3.5, 1.25},
  {3, -0.25}, {2, -0.25},
  {1.5, 0.25}, {1.5, 4.5},
  {2, 4}, {3, 5},
  {3.5, 4.5}, {3.5, 2.5}
}
waypoints.lane_outer =  {
  {4.0, 2.5}, {4.0, 4.5},
  {3.0, 5.75}, {1.75, 5.75},
  {0.75, 5.0},
  {0.75, 0}, -- begin turn
  {1.25, -0.75}, {3.0, -0.75},
  {4.0, 0.0}, {4.0, 2.0}
}
-- Upper loop
waypoints.lane_outerA =  {
  {4.0, 3.0}, {4.0, 4.5},
  {3.0, 5.75}, {1.75, 5.75},
  {0.75, 5.0},
  {0.75, 3.5}, -- begin turn
  {1.25, 2.5}, {2.75, 2.5},
  {4.0, 3.0},
}
-- Lower loop
-- waypoints.lane_outerB =  {
--   {4.0, 2.5}, {4.0, 4.5},
--   {3.0, 5.75}, {1.75, 5.75},
--   {0.75, 5.0}, {0.75, 0},
--   {1.25, -0.75}, {3.0, -0.75},
--   {4.0, 0.0}, {4.0, 2.0}
-- }

waypoints.lane_enter = {
  {-2.5, 1.5}, {-1.25, 1.5}
}
waypoints.turn_left = {
  {-1.5, 1.5}, {-0.8, 1.5}, {-0.7705948579011315, 1.5014445819983409}, {-0.7414729033951611, 1.5057644158790309}, {-0.712914596823661, 1.5129178992803374}, {-0.6851949702904727, 1.5228361402466142}, {-0.6585809789522005, 1.5354236206954937}, {-0.6333289300941191, 1.5505591163092367}, {-0.6096820147509061, 1.5680968639911792}, {-0.5878679656440355, 1.587867965644036}, {-0.5680968639911788, 1.6096820147509066}, {-0.5505591163092364, 1.6333289300941196}, {-0.5354236206954934, 1.6585809789522008}, {-0.522836140246614, 1.6851949702904732}, {-0.5129178992803374, 1.7129145968236614}, {-0.5057644158790309, 1.7414729033951617}, {-0.5014445819983409, 1.770594857901132}, {-0.5, 1.8}, {-0.5, 3}
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
  paths[k].ds = ds
end

-- Given car pose
local function find_lane(p_vehicle, closeness)
  closeness = tonumber(closeness) or 1
  local position = {unpack(p_vehicle, 1, 2)}

  local candidates = {}
  for k, path in pairs(paths) do
    if k:match"^lane_" then
      local nearest, err = path.tree:nearest(position, closeness)
      nearest = nearest and nearest[1]
      if nearest then
        candidates[k] = {
          id_path = nearest.user,
          dist = math.sqrt(nearest.dist_sq)
        }
      end
    end
  end

  -- Evaluate our candidates
  local dir = {math.cos(p_vehicle.a), math.sin(p_vehicle.a)}
  local dot_threshold = math.sqrt(2) / 2
  local dmin, kmin, imin = math.huge, nil, nil
  for k, cand in pairs(candidates) do
    local i = cand.id_path
    -- Note that path points are vectors already
    local path = paths[k]
    local p_before = path[i-1]
    local p_after = path[i+1]
    local dp
    if p_before and p_after then
      dp = p_after - p_before
    elseif p_before then
      dp = path[i] - p_before
    elseif p_after then
      dp = p_after - path[i]
    end
    local dot = vector.dot(dir, vector.unit(dp))
    -- if dot > dot_threshold and cand.dist < dmin then
    if cand.dist < dmin then
      kmin = k
      imin = cand.id_path
      dmin = cand.dist
    end
  end

  if kmin then
    return {name_path = kmin, id_path=imin, dist=dmin}
  else
    return false, "No path found"
  end
end

local cofsm = require'cofsm'
local fsm_control = cofsm.new{
  -- Go is a sink, that goes to the next state
  -- Should find lane, etc.
  {'botStop', 'go', 'botGo'},
  -- Stop is a sink
  {'botGo', 'stop', 'botStop'},
  -- Follow the lane loop
  {'botFollowLane', 'stop', 'botStop'},
  {'botFollowLane', 'done', 'botFollowLane'},
  {'botGo', 'lane', 'botFollowLane'},
  -- Turn at the intersection
  {'botTurn', 'stop', 'botStop'},
  {'botTurn', 'done', 'botGo'}, -- botGo will find a lane
  {'botGo', 'turn', 'botTurn'},
  -- Approach the intersection
  {'botApproach', 'stop', 'botStop'},
  {'botApproach', 'done', 'botTurn'},
  {'botApproach', 'turn', 'botTurn'},
  {'botGo', 'approach', 'botApproach'},
}

-- Need be an exit function only, really
local function get_turn_path(path_name)
  my_path = paths[path_name or desired_turn]
  co_control = cocreate(control.pure_pursuit{
                        path=my_path,
                        lookahead=lookahead})
end

local function get_outer_path(path_name)
  print("path_name input", path_name)
  local random_path_id = math.random(2)
  if random_path_id == 2 then
    path_name = 'lane_outerA'
  else
    path_name = 'lane_outer'
  end
  print("path_name random", path_name)
  my_path = paths[path_name]
  co_control = cocreate(control.pure_pursuit{
                        path=my_path,
                        fn_nearby=fn_nearby,
                        lookahead=lookahead,
                        id_start=1})
  return path_name
end

-- Keep looping
local function loop_path()
  co_control = cocreate(control.pure_pursuit{
                        path=my_path,
                        fn_nearby=fn_nearby,
                        lookahead=lookahead,
                        id_start=1
                        })
end
local function update_steering(pose_rbt)
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
    print('DONE!', fsm_control.current_state)
    if fsm_control.current_state=='botApproach' then
      if risk.d_j and risk.d_j <= 0 then
        get_turn_path("lane_enter")
      else
        get_turn_path()
      end
    else
      -- loop_path()
      my_path_name = get_outer_path(my_path_name)
    end
    fsm_control:dispatch"done"
    return
  end

  local steering = math.atan(result.kappa * wheel_base)
  result.steering = steering
  return result
end


local function vicon2pose(vp)
  return vp.translation[1] / 1e3, vp.translation[2] / 1e3, vp.rotation[3]
end

local last_frame = -math.huge
local function parse_vicon(msg)
  -- Check that the data is not stale
  -- TODO: Stale for each ID...
  local frame = msg.frame
  if frame < last_frame then
    return false, "Stale data"
  else
    last_frame = frame
    msg.frame = nil
  end

  -- Find the pose for each robot
  for id, vp in pairs(msg) do
    poses[id] = vector.pose{vicon2pose(vp)}
  end

end

local function parse_risk(msg)
  risk = msg
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

local function parse_joystick(msg)
  if type(msg.buttons)~='table' or type(msg.axes)~='table' then
    return
  end
  -- Set the human velocity
  if msg.axes[2] then
    vel_h = vel_max * msg.axes[2] / -32767
  end
end

local cb_tbl = {
  vicon = parse_vicon,
  risk = parse_risk,
  houston = parse_houston,
  joystick = parse_joystick,
}

-- TODO: Check the relative pose between us and that ID
local function update_lead(my_id)
  my_id = my_id or id_rbt

  for id, p in pairs(poses) do
    local lane, err = find_lane(p)
    if lane then
      lanes[id] = lane
    else
      print("update_lead | find_lane", err)
    end
  end
  local my_lane = lanes[my_id]
  if not my_lane then return false, "Not in a lane" end

  local lead_offset, id_lead = math.huge, nil
  for id, lane in pairs(lanes) do
    -- print("Lead ID", id, my_id)
    if id~=my_id and lane.name_path==my_lane.name_path then
      -- TODO: ds should be specific to the path...
      local path_offset = (lane.id_path - my_lane.id_path) * ds
      if path_offset > 0 and path_offset < lead_offset then
        id_lead = id
        lead_offset = path_offset
      end
    end
  end

  if not id_lead then
    return false, "No lead vehicle"
  end

  return id_lead, lead_offset
end

-- local function botGo()
--   while true do
--     print('Entry', 'botGo')
--     -- Done Entry
--     coroutine.yield()
--     -- Update cycle
--     repeat
--       -- print('Update', 'go', sensors_latest)
--       local evt = 'stop'
--       local is_done = coroutine.yield(evt)
--     until is_done
--     -- Exit Mode
--     print('Exit', 'botGo')
--     -- Done Exit
--     coroutine.yield()
--   end
-- end

local function cb_loop(t_us)
  -- Update the fsm
  -- fsm_control:update()

  -- Find our lane
  -- local lane_rbt = lanes[id_rbt]

  -- Use human velocity or autonomous velocity (lane velocity)
  local vel_v = vel_h or vel_l

  -- For sending to the vesc
  local result = {}
  local my_state = fsm_control.current_state
  -- if my_state ~= 'botStop' then
  --   print("my_state", my_state)
  -- end

  if my_state == 'botStop' then
    vel_v = 0
  elseif my_state == 'botGo' then
    -- Find the correct path and then transition
    local pose_rbt = poses[id_rbt]
    if pose_rbt then
      local info_lane, err = find_lane(pose_rbt)
      if info_lane then
        my_path_name = info_lane.name_path
        my_path = paths[info_lane.name_path]
        print("info_lane.name_path", info_lane.name_path)
        co_control = cocreate(control.pure_pursuit{
                              path=my_path,
                              fn_nearby=fn_nearby,
                              lookahead=lookahead})
        if info_lane.name_path == 'lane_enter' then
          fsm_control:dispatch"approach"
        elseif info_lane.name_path:match'^lane_' then
          fsm_control:dispatch"lane"
        end
      -- else
      --   print('info_lane', err)
      end
    end
  elseif my_state == 'botFollowLane' then
    -- Go at least the lane speed
    vel_v = math.min(vel_l, vel_v)
    local pose_rbt = poses[id_rbt]
    result = update_steering(pose_rbt)
    -- Find a lead vehicle
    local id_lead, lead_offset = update_lead()
    if id_lead then
      -- Slow for a lead vehicle
      local d_stop = 0.8
      local d_near = 1.5
      local ratio = (lead_offset - d_stop) / (d_near - d_stop)
      ratio = max(0, min(ratio, 1))
      vel_v = ratio * vel_v
      if lead_offset < d_near then
        print(string.format("Stopping for %s | [%.2f -> %.2f]",
                            id_lead, ratio, vel_v))
      end
    -- else
    --   print("id_lead", lead_offset)
    end
  elseif my_state == 'botApproach' then
    local pose_rbt = poses[id_rbt]
    result = update_steering(pose_rbt)
    -- Make sure we go fast enough to move...
    if risk.d_j then
      print("go", risk.go, "d_j", risk.d_j, "vel_v", vel_v)
      if math.abs(vel_v) >= vel_min then
        local ratio = math.abs(risk.d_j) / 1.6
        vel_v = vel_v * max(0, min(ratio, 1))
        if vel_v < vel_min and vel_v>=0 then
          vel_v = vel_min
        elseif vel_v > -vel_min and vel_v <= 0 then
          vel_v = -vel_min
        end
      end
      -- Only going backwards if no go
      if not risk.go and math.abs(risk.d_j) < 0.11 then
        vel_v = math.min(0, vel_v)
      end
    end
    print("botApproach velocity", vel_v)
  elseif my_state == 'botTurn' then
    local pose_rbt = poses[id_rbt]
    result = update_steering(pose_rbt)
    -- Check t_clear
    if risk.max_t_clear then
      local min_vel_clear = paths.turn_left.length / risk.max_t_clear
      print("min_vel_clear", min_vel_clear)
      vel_v = math.max(vel_v, min_vel_clear)
    end
    print("botTurn velocity", vel_v)
  else
    print("Unknown state!")
    fsm_control:dispatch"stop"
  end
  -- print('result.rpm', result.rpm)
  vel_v = math.max(-vel_max, math.min(vel_v, vel_max))

  result = result or {}
  result.rpm = vel_v * racecar.RPM_PER_MPS

  result.state = my_state

  local ret, err = log_announce(log, result, "control")
  if not ret then
    print("Error!", err)
  end

end

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = 100, -- 100ms loop
  fn_loop = cb_loop
}

if log then log:close() end
