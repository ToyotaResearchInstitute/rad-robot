#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local log_announce = racecar.log_announce

local DEBUG_ANNOUNCE = os.getenv("ANNOUNCE") or flags.announce
local RUN_SIMULATION = os.getenv("SIMULATE") or flags.simulate
local id_robot = flags.id or racecar.HOSTNAME
assert(id_robot, "No name provided!")
print("RUN_SIMULATION", RUN_SIMULATION)
math.randomseed(123)

local grid = require'grid'
local atan = require'math'.atan
local atan2 = require'math'.atan2 or require'math'.atan
local transform = require'transform'
local tf2D_inv = require'transform'.tf2D_inv
local usleep = require'unix'.usleep
local vector = require'vector'

local has_logger, logger = pcall(require, 'logger')

local log = has_logger and flags.log ~= 0 and assert(logger.new('control', racecar.ROBOT_HOME.."/logs"))

-- Test the control library
local control = require'control'
local generate_control_points = require'control'.generate_control_points
local generate_path = require'control'.generate_path

-- Globally accessible variables
local veh_poses = {}
local desired_path = flags.path or 'outer'
local vel_h = false
local vel_lane = 0.5
local vel_max = 0.75 -- 1 --0.75
local vel_min = 0.2
-- Simulation parameters
local wheel_base = 0.3
-- Parameters for trajectory following
local dt = 0.1
local lookahead = 0.33
-- local lookahead = 0.6
local threshold_close = 0.25
--
local function cb_debug(t_us)
  local pose_rbt = veh_poses[id_robot]
  if not pose_rbt then return end
  local px, py, pa = unpack(pose_rbt)
  local info = {
    string.format("Path: %s", desired_path),
    string.format("Pose: x=%.2fm, y=%.2fm, a=%.2f°", px, py, math.deg(pa))
  }
  return table.concat(info, "\n")
end

-- Path increments: one inch
local ds = 0.10

-- For grid drawing
local function set_mid(map, idx) map[idx] = 127 end
local function set_quarter(map, idx) map[idx] = 63 end

-- Holodeck Grid
local g_holo = assert(grid.new{
  scale = 0.01,
  xmin = 0, xmax = 4.5,
  ymin = -1, ymax = 6
})

local g_loop = assert(grid.new{
  scale = 0.01,
  xmin = 0, xmax = 4.5,
  ymin = -1, ymax = 6
})

local routes = {}
-- Inner and outer are merely two different lanes
-- On the same road
routes.inner = {
  {1.25, -0.25, math.rad(90)},
  {1.25, 5.25, math.rad(0)},
  {3.5, 5.25, math.rad(270)},
  {3.5, -0.25, math.rad(180)},
  turning_radius = 0.3,
  closed = true
}
routes.outer = {
  {0.75, -0.75, math.rad(0)},
  {4, -0.75, math.rad(90)},
  {4, 5.75, math.rad(180)},
  {0.75, 5.75, math.rad(270)},
  turning_radius = 0.3,
  closed = true
}
-- Smaller loops
--[[
routes.outerA = {
  {0.75, -0.75, math.rad(0)},
  {4, -0.75, math.rad(90)},
  {4, 2.5, math.rad(180)},
  {0.75, 2.5, math.rad(270)},
  turning_radius = 0.3,
  closed = true
}
--]]
--[[
routes.outerB = {
  {4, 5.75, math.rad(180)},
  {0.75, 5.75, math.rad(270)},
  {0.75, 2.5, math.rad(0)},
  {4, 2.5, math.rad(90)},
  turning_radius = 0.3,
  closed = true
}
--]]

-- Generate the knots
-- These show the points before and after a turn
local route_knots = {}
for name, route in pairs(routes) do
  local knots = assert(generate_control_points(route))
  route_knots[name] = knots
end
-- Print the knots
for name, knots in pairs(route_knots) do
  print("Route knots", name)
  for i, kn in ipairs(knots) do print(i, unpack(kn)) end
end

-- Intersection lookup helper
local all_knots = {
  tree = require'kdtree'.create(2),
  last = {n = 0}
}
for name, knots in pairs(route_knots) do
  print(name, "n_knots", #knots)
  for i, knot in ipairs(knots) do
    local info = {name, i}
    table.insert(all_knots, info)
    -- Insert with the ID to the route information
    all_knots.tree:insert(knot, #all_knots)
  end
end

-- Go from a route to a list of points (path)
local paths = {}
for name, knots in pairs(route_knots) do
  g_holo:fill(0)
  local path, length = generate_path(knots, {
  ds = ds, grid_raster = g_holo, closed = routes[name].closed})
  assert(path, length)
  assert(#path > 0, "No points in path")
  assert(length > 0, "No path length")
  -- Since we are drawing, save the drawing of the path(s)
  assert(g_holo:save("/tmp/path_"..name..".pgm"))
  -- Add to the table of paths
  path.length = length
  paths[name] = path
  print(string.format("Route [%s] Length [%.2f meters] Points [%d]",
  name, path.length, #path))
end

-- TODO: Paths should come from a separate program
my_path = assert(paths[desired_path], "Path not found!")
local pp_params = {
  path = my_path,
  lookahead = lookahead,
  threshold_close = threshold_close
}
local pp = assert(control.pure_pursuit(pp_params))

-- Set the environment for displaying in-browser
local env = {
  viewBox = {g_holo.xmin, g_holo.ymin, g_holo.xmax - g_holo.xmin, g_holo.ymax - g_holo.ymin},
  observer = pose_rbt,
  time_interval = dt,
  speed = vel_lane,
  -- Show the knots for better printing
  lanes = {
    -- {unpack(route_knots.inner)}, {unpack(route_knots.outer)}
    {unpack(paths.inner)}, {unpack(paths.outer)},
    -- {unpack(paths.outerA)}, {unpack(paths.outerB)}
  },
  -- This isn't quite right...?
  trajectory_turn = {
    -- {unpack(paths.inner)}, {unpack(paths.outer)}
  },
}

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  -- Check which lane my vehicle is in
  -- Check in which lanes all of the cars are
  local in_my_lane = {}
  for name_veh, pose_veh in pairs(veh_poses) do
    if name_veh ~= id_robot then
      local info = control.find_in_path(pose_veh, paths, 0.5)
      -- Check if they are in our lane
      if info then
        if info.path_name==desired_path then
          table.insert(in_my_lane, name_veh)
        end
      end
    end
  end
  local pose_rbt = veh_poses[id_robot]
  if not pose_rbt then
    -- Stop if no pose...
    local control_inp = {id = id_robot, steering = 0, velocity = 0}
    log_announce(log, control_inp, "control")
    return
  end
  if #in_my_lane > 0 then
    print("Monitoring my lane: ", table.concat(in_my_lane, ", "))
    local p_x, p_y, p_a = unpack(pose_rbt)
    for _, name_veh in ipairs(in_my_lane) do
      local x_veh, y_veh, a_veh = unpack(veh_poses[name_veh])
      -- TODO: Relative w.r.t. to lane geometry, not the car
      local dx_rel, dy_rel = tf2D_inv(x_veh, y_veh, p_a, p_x, p_y)
      print("dx_rel", dx_rel)
      if dx_rel < 5 * wheel_base and dx_rel > 0 then
        print("Lane leader | ", name_veh, dx_rel, dy_rel)
      else
        print("Not leader | ", name_veh, dx_rel, dy_rel)
      end
    end
  end

  -- Find our control policy
  local result, err = pp(pose_rbt)
  if not result then return false, err end
  -- for k, v in pairs(result) do
  --   if type(v)=='table' then
  --     print(k, table.concat(v, ', '))
  --   else
  --     print(k, v)
  --   end
  -- end
  if result.err then
    return false, result.err
  end
  -- Ensure we add our ID to the result
  result.id = id_robot
  -- Save the pose, just because
  result.pose = pose_rbt
  -- Set the steering based on the car dimensions
  local steering = atan(result.kappa * wheel_base)
  result.steering = steering
  -- TODO: Set the speed based on curvature? Lookahead point's curvature?
  result.velocity = vel_lane

  -- TODO: Call our simulation within here? Speed/Dropped packets
  -- if RUN_SIMULATION then
  --   local control_inp = {steering = steering, velocity = vel_lane}
  --   local state = assert(simulate_vehicle({pose=pose_rbt}, control_inp))
  --   pose_rbt = state.pose
  -- end
  -- Should we broadcast?
  if DEBUG_ANNOUNCE then
    log_announce(log, result, "control")
    -- TODO: Environment should be in a different Lua file, as it listen to the path
    -- env.observer = pose_rbt
    assert(racecar.announce("risk", env))
    -- Wait a touch if simulating
    if RUN_SIMULATION then usleep(1e4) end
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

local cb_tbl = {
  vicon = parse_vicon,
}

local function exit()
  if log then log:close() end
  if DEBUG_ANNOUNCE then
    racecar.announce("control", {id = id_robot, steering = 0, velocity = 0})
  end
  -- assert(g_holo:save"/tmp/simulated.pgm")
  -- assert(g_loop:save(string.format("/tmp/loop%02d.pgm", loop_counter)))
  -- print("p_history", #p_history)
  -- assert(require'fileformats'.save_ply("/tmp/simulation.ply", p_history))
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
