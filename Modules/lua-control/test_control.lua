#!/usr/bin/env luajit
local coroutine = require'coroutine'
local unpack = unpack or require'table'.unpack

local control = require'control'
local transform = require'transform'
local vector = require'vector'
local racecar = require'racecar'
local usleep = require'unix'.usleep
local kdtree = require'kdtree'

local speed = 0.1 -- meteres per second
local dt = 0.1
local dheading_max = 45 * racecar.DEG_TO_RAD -- radians of front wheels
local iloop = 0
local wheel_base = 0.3
local lookahead = 0.4
local threshold_close = 1.2

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
  {-0.5, 3.0}
}
waypoints.traj_right_turn = {
  {-2, 1.5},
  {-1.5, 1.5}, {-1.5205948579011328, 1.4985554180016591}, {-1.4914729033951626, 1.4942355841209694}, {-1.4629145968236625, 1.4870821007196628}, {-1.4351949702904743, 1.4771638597533865}, {-1.408580978952202, 1.464576379304507}, {-1.3833289300941205, 1.4494408836907644}, {-1.3596820147509077, 1.4319031360088221}, {-1.3378679656440369, 1.4121320343559653}, {-1.31809686399118, 1.390317985249095}, {-1.3005591163092374, 1.3666710699058822}, {-1.2854236206954943, 1.341419021047801}, {-1.2728361402466148, 1.3148050297095288}, {-1.262917899280338, 1.2870854031763408}, {-1.2557644158790313, 1.2585270966048407}, {-1.251444581998341, 1.2294051420988705}, {-1.25, 1.2},
  {-1.25, -1.0},
}

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
  paths[k] = path
  print("Path", k, unpack(path))
  local tree = kdtree.create(2)
  for i, p in ipairs(path) do tree:insert(p, i) end
  paths[k].tree = tree
end

local desired_path = 'traj_right_turn'
local my_path = paths[desired_path]
local pose_rbt = vector.pose()
pose_rbt.x, pose_rbt.y = unpack(my_path[1])
pose_rbt.a = math.atan2(my_path[2][2]-pose_rbt.y, my_path[2][1]-pose_rbt.x)

local env = {
  viewBox = {-3, -5.5, 7, 9},
  observer = pose_rbt,
  time_interval = 0.1,
  speed = 0.1,
  lanes = {waypoints.lane_inner, waypoints.lane_outer},
  trajectory_turn = {waypoints.traj_left_turn, waypoints.traj_right_turn},
}

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

local co = coroutine.create(control.pure_pursuit{
  path=my_path,
  fn_nearby=fn_nearby,
  lookahead=lookahead,
  -- id_start = 1
})


while racecar.running and iloop < 2000 do
  print("= Iteration", iloop)
  -- print(env)
  print("Pose", pose_rbt)
  local running, result, err = coroutine.resume(co, pose_rbt)
  if not running then
    print("Not running", result)
    racecar.announce("control", {steering = 0, velocity = 0})
    return os.exit()
  elseif type(result)~='table' then
    print("Improper", result, err)
    return
  elseif result.done then
    -- Keep looping
    print("Done!")
    return
  elseif result.far then
    print("far!")
    return
  end
  print("Path", result.id_path)
  print("p_nearby", result.p_nearby)
  print("p_lookahead", unpack(result.p_lookahead))
  print("kappa", result.kappa)

  -- Set the angle of the front wheels
  local steering = math.atan(result.kappa * wheel_base)
  print("Steering", steering * racecar.RAD_TO_DEG)

  -- Kinematic model propagation
  steering = math.min(math.max(-dheading_max, steering), dheading_max)
  local da = (speed * dt) / wheel_base * math.tan(steering)
  local dx, dy = transform.rot2D(speed * dt, 0, pose_rbt[3])
  local dpose = vector.pose{dx, dy, da}

  print("dpose", dpose)

  pose_rbt.x = pose_rbt.x + dpose.x
  pose_rbt.y = pose_rbt.y + dpose.y
  local a1 = pose_rbt.a + dpose.a
  if a1 > math.pi then
    a1 = a1 - 2 * math.pi
  elseif a1 < -math.pi then
    a1 = a1 + 2 * math.pi
  end
  pose_rbt.a = a1

  env.observer = pose_rbt
  racecar.announce("risk", env)
  result.steering = steering
  result.velocity = speed
  racecar.announce("control", result)
  usleep(1e4)
  iloop = iloop + 1
end
