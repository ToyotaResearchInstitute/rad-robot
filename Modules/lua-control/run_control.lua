#!/usr/bin/env luajit
local coresume = require'coroutine'.resume
local cocreate = require'coroutine'.create
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local desired_path = flags.desired or 'lane_outer'

local control = require'control'
local has_logger, logger = pcall(require, 'logger')
local vector = require'vector'
local log_announce = racecar.log_announce
local log = has_logger and flags.log~=0
            and assert(logger.new('control', flags.home.."/logs"))

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
waypoints.traj_left_turn = {
  {-2, 1.5},
  {-0.8, 1.5}, {-0.7705948579011315, 1.5014445819983409}, {-0.7414729033951611, 1.5057644158790309}, {-0.712914596823661, 1.5129178992803374}, {-0.6851949702904727, 1.5228361402466142}, {-0.6585809789522005, 1.5354236206954937}, {-0.6333289300941191, 1.5505591163092367}, {-0.6096820147509061, 1.5680968639911792}, {-0.5878679656440355, 1.587867965644036}, {-0.5680968639911788, 1.6096820147509066}, {-0.5505591163092364, 1.6333289300941196}, {-0.5354236206954934, 1.6585809789522008}, {-0.522836140246614, 1.6851949702904732}, {-0.5129178992803374, 1.7129145968236614}, {-0.5057644158790309, 1.7414729033951617}, {-0.5014445819983409, 1.770594857901132}, {-0.5, 1.8},
  {-0.5, 3.75}
}
waypoints.traj_right_turn = {
  {-2, 1.5},
  {-1.5, 1.5}, {-1.5205948579011328, 1.4985554180016591}, {-1.4914729033951626, 1.4942355841209694}, {-1.4629145968236625, 1.4870821007196628}, {-1.4351949702904743, 1.4771638597533865}, {-1.408580978952202, 1.464576379304507}, {-1.3833289300941205, 1.4494408836907644}, {-1.3596820147509077, 1.4319031360088221}, {-1.3378679656440369, 1.4121320343559653}, {-1.31809686399118, 1.390317985249095}, {-1.3005591163092374, 1.3666710699058822}, {-1.2854236206954943, 1.341419021047801}, {-1.2728361402466148, 1.3148050297095288}, {-1.262917899280338, 1.2870854031763408}, {-1.2557644158790313, 1.2585270966048407}, {-1.251444581998341, 1.2294051420988705}, {-1.25, 1.2},
  {-1.25, -1.75},
}

local lookahead = 0.5
local wheel_base = 0.3

local ds = 0.05
local paths = {}
for k, wps in pairs(waypoints) do
  -- Two dimensional points
  local path = {}
  for i=1, #wps-1 do
    local p_a = vector.new(wps[i])
    local p_b = vector.new(wps[i+1])
    local dp = p_b - p_a
    local d = vector.norm(dp)
    dp = vector.unit(dp)
    table.insert(path, p_a)
    for step = ds, d-ds, ds do
      local p = p_a + step * dp
      table.insert(path, p)
    end
  end
  local tree = kdtree.create(2)
  for i, p in ipairs(path) do tree:insert(p, i) end
  paths[k] = path
  paths[k].tree = tree
end

----------------------
local my_path = assert(paths[desired_path], "No desired path found: "..tostring(desired_path))
local env = {
  viewBox = {-3, -5.5, 7, 9},
  observer = vector.pose(),
  time_interval = 0.1,
  speed = 0.1,
  lanes = {waypoints.lane_inner, waypoints.lane_outer},
  trajectory_turn = {waypoints.traj_left_turn, waypoints.traj_right_turn},
}

local co_control = cocreate(control.pure_pursuit(my_path, lookahead))

-- Given car pose
local function find_lane(p_vehicle)
  local kmin, dmin, imin = false, math.huge, nil
  for k, path in pairs(paths) do
    if k:match"^lane" then
      local nearest, err = path.tree:nearest({unpack(p_vehicle, 1, 2)}, 1)
      nearest = nearest and nearest[1]
      -- TODO: Check dot product direction
      if not nearest then
        print("Not near!", err, path.tree:size(), k, p_vehicle)
      else
        print("Near", k, nearest.dist_sq)
        if nearest.dist_sq < dmin then
          imin = nearest.user
          dmin = nearest.dist_sq
          kmin = k
        end
      end
    end
  end
  return {name_path = kmin, id_path=imin, dist_sq=dmin}
end

local function vicon2pose(vp)
  return vp.translation[1] / 1e3, vp.translation[2] / 1e3, vp.rotation[3]
end

local my_id = 'tri1'
local last_frame = -math.huge
local function parse_vicon(msg)
  if msg.frame < last_frame then
    return false, "Stale data"
  end
  local poses, lanes = {}, {}
  for id, vp in pairs(msg) do
    if id~='frame' then
      local p = vector.pose{vicon2pose(vp)}
      poses[id] = p
      print("Find", id)
      lanes[id] = find_lane(p)
    end
  end
  -- Update the robot pose
  local pose_rbt = poses[my_id]
  print("Pose", pose_rbt)
  if not pose_rbt then return end
  -- Check if a car is in my lane :)
  local my_lane = lanes[my_id]
  print("My Lane", my_lane.name_path, my_lane.id_path, my_lane.dist_sq)
  lanes[my_id] = nil

  local lead_offset = math.huge
  for id, lane in pairs(lanes) do
    if lane.name_path==my_lane.name_path then
      -- TODO: Check the relative pose between us and that ID
      local path_offset = (lane.id_path - my_lane.id_path) * ds
      print(id, "in my lane", lane.id_path, "distance", path_offset)
      if path_offset > 0 then
        lead_offset = math.min(lead_offset, path_offset)
      end
    else
      print(id, "not in my lane", lane.name_path)
    end
  end

  local running, result, err = coresume(co_control, pose_rbt)
  if not running then
    print("Not running", result)
    log_announce(log, { steering = 0, velocity = 0 }, "control")
    return os.exit()
  elseif type(result)~='table' then
    print("Improper", result, err)
    log_announce(log, { steering = 0, velocity = 0 }, "control")
    return
  elseif result.done then
    -- Keep looping
    print("Restarting")
    co_control = cocreate(control.pure_pursuit(my_path, lookahead))
    return
  end

  local steering = math.atan(result.kappa * wheel_base)
  print("Path", result.id_path, my_path[result.id_path])
  print("Steering angle", steering * racecar.RAD_TO_DEG)

  if lead_offset < 0.9 then
    print("Stop to not hit!!")
  end
  env.observer = pose_rbt
  -- For GUI plotting
  log_announce(log, env, "risk")
  -- For sending to the vesc
  result.steering = steering
  result.velocity = 8 -- duty cycle

  if lead_offset < 0.8 then
    result.velocity = 0
  elseif lead_offset < 1.5 then
    result.velocity = (lead_offset - 0.8) / (1.5 - 0.8) * result.velocity
  end

  log_announce(log, result, "control")
end

racecar.listen{
  vicon = parse_vicon
}
