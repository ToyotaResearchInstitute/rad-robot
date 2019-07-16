#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack

local racecar = require'racecar'
racecar.init()
local flags = racecar.parse_arg(arg)
local id_robot = flags.id or racecar.HOSTNAME
assert(id_robot, "No name provided!")
math.randomseed(123)

local grid = require'grid'
local vector = require'vector'

local log_announce = racecar.log_announce
local has_logger, logger = pcall(require, 'logger')
local log = has_logger and flags.log ~= 0 and assert(logger.new('plan', racecar.ROBOT_HOME.."/logs"))

-- Test the control library
local path = require'path'
local generate_waypoints = require'path'.generate_waypoints
local path_from_waypoints = require'path'.path_from_waypoints

-- Globally accessible variables
local veh_poses = {}
local desired_path = flags.path or 'outer'
local vel_lane0 = tonumber(flags.vel_lane) or 0.5
local vel_lane = vel_lane0

-- Parameters for trajectory following
local dt = 0.1

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

-- Holodeck Grid
local g_holo = assert(grid.new{
  scale = 0.01,
  xmin = 0, xmax = 4.5,
  ymin = -1, ymax = 6
})

local routes = {}
-- Inner and outer are merely two different lanes
-- On the same road
-- routes.lane_outer = {
--   {0.75, -0.75, math.rad(0)},
--   {4, -0.75, math.rad(90)},
--   {4, 5.75, math.rad(180)},
--   {0.75, 5.75, math.rad(270)},
--   turning_radius = 0.3,
--   closed = true
-- }

routes['lane_outerA+1'] = {
  {0.75, 2.25, math.rad(270)},
  {0.75, -0.75, math.rad(0)},
  {4, -0.75, math.rad(90)},
  {4, 2.25, math.rad(180)},
  turning_radius = 0.3,
  closed = false
}
routes['lane_outerB+1'] = {
  {4, 2.75, math.rad(90)},
  {4, 5.75, math.rad(180)},
  {0.75, 5.75, math.rad(270)},
  {0.75, 2.75, math.rad(0)},
  turning_radius = 0.3,
  closed = false
}

routes['lane_innerA+1'] = {
  {1.25, 3.25, math.rad(90)},
  {1.25, 5.25, math.rad(0)},
  {3.5, 5.25, math.rad(270)},
  {3.5, 3.25, math.rad(180)},
  turning_radius = 0.3,
  closed = false
}

routes['lane_innerB+1'] = {
  {3.5, 1.75, math.rad(270)},
  {3.5, -0.25, math.rad(180)},
  {1.25, -0.25, math.rad(90)},
  {1.25, 1.75, math.rad(0)},
  turning_radius = 0.3,
  closed = false
}

-- Middle lanes
routes['lane_middleA+1'] = {
  {1.75, 2.25, math.rad(0)},
  {3.0, 2.25, math.rad(0)},
  turning_radius = 0.3,
  closed = false
}
routes['lane_middleA-1'] = {
  {3.0, 2.75, math.rad(180)},
  {1.75, 2.75, math.rad(180)},
  turning_radius = 0.3,
  closed = false
}

-- Start turns
routes['turn_outerB+1_outerA+1'] = {
  {0.75, 2.75, math.rad(270)},
  {0.75, 2.25, math.rad(270)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_outerA+1_outerB+1'] = {
  {4.0, 2.25, math.rad(90)},
  {4.0, 2.75, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}
--
routes['turn_innerB+1_innerA+1'] = {
  {1.25, 1.75, math.rad(270)},
  {1.25, 3.25, math.rad(270)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_innerA+1_innerB+1'] = {
  {3.5, 3.25, math.rad(90)},
  {3.5, 1.75, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}

-- Turn options from outer
routes['turn_outerB+1_middleA+1'] = {
  {0.75, 2.75, math.rad(270)},
  {0.75, 2.25, math.rad(0)},
  {1.75, 2.25, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_outerA+1_middleA-1'] = {
  {4, 2.25, math.rad(90)},
  {4, 2.75, math.rad(180)},
  {3.0, 2.75, math.rad(270)},
  turning_radius = 0.3,
  closed = false
}

-- From inner
routes['turn_innerB+1_middleA+1'] = {
  {1.25, 1.75, math.rad(90)},
  {1.25, 2.25, math.rad(0)},
  {1.75, 2.25, math.rad(0)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_innerA+1_middleA-1'] = {
  {3.5, 3.25, math.rad(270)},
  {3.5, 2.75, math.rad(180)},
  {3.0, 2.75, math.rad(180)},
  turning_radius = 0.3,
  closed = false
}

-- From middle to inner
routes['turn_middleA-1_innerA+1'] = {
  {1.75, 2.75, math.rad(0)},
  {1.25, 2.75, math.rad(0)},
  {1.25, 3.25, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_middleA-1_outerA+1'] = {
  {1.75, 2.75, math.rad(0)},
  {0.75, 2.75, math.rad(0)},
  {0.75, 2.25, math.rad(270)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_middleA+1_innerB+1'] = {
  {3.0, 2.25, math.rad(0)},
  {3.5, 2.25, math.rad(0)},
  {3.5, 1.75, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}
routes['turn_middleA+1_outerB+1'] = {
  {3.0, 2.25, math.rad(0)},
  {4, 2.25, math.rad(0)},
  {4, 2.75, math.rad(90)},
  turning_radius = 0.3,
  closed = false
}


-- local radius_roundabout1 = 1.5
-- local radius_roundabout2 = 2.5
-- routes.roundabout1 = {
--   {2.5 + radius_roundabout, 2.5, math.rad(90)},
--   {2.5, 2.5 + radius_roundabout, math.rad(180)},
--   {2.5 - radius_roundabout, 2.5, math.rad(270)},
--   {2.5, 2.5 - radius_roundabout, math.rad(0)},
--   turning_radius = radius_roundabout1,
--   closed = true
-- }

-- routes.roundabout2 = {
--   {4, -0.75, math.rad(90)},
--   {4, 5.75, math.rad(180)},
--   {0.75, 5.75, math.rad(270)},
--   {0.75, -0.75, math.rad(0)},
--   turning_radius = 0.3,
--   closed = true
-- }

-- routes.merge = {
--   -- Starting point
--   {-1.0, 2.5, math.rad(0)},
--   {1.5, 2.5, math.rad(15)},
--   {2.0, 2.75, math.rad(30)},
--   -- {3.0, 3.0, math.rad(15)}, -- Just before merging
--   {3.5, 3.25, math.rad(15)}, -- Merged
--   {5.0, 3.25, math.rad(0)}, -- Merged
--   turning_radius = 0.3,
--   closed = false
-- }

-- routes.highway = {
--   -- Starting point
--   {-1.0, 3.25, math.rad(0)},
--   {5.0, 3.25, math.rad(0)}, -- Merged
--   {6.0, 4.25, math.rad(90)},
--   {5.0, 5.25, math.rad(180)},
--   {0.0, 5.25, math.rad(180)},
--   {-1.0, 4.25, math.rad(270)},
--   turning_radius = 0.3,
--   closed = true
-- }



-- Generate the knots
-- These show the points before and after a turn
local route_knots = {}
for name, route in pairs(routes) do
  local knots = assert(generate_waypoints(route))
  route_knots[name] = knots
end
-- Print the knots
for name, knots in pairs(route_knots) do
  print("Route", name)
  for i, kn in ipairs(knots) do print(i, unpack(kn)) end
end

-- Go from a route to a list of points (path)
local paths = {}
for name, knots in pairs(route_knots) do
  g_holo:fill(0)
  local points, length = path_from_waypoints(knots, {
  ds = ds, grid_raster = g_holo, closed = routes[name].closed})
  assert(points, length)
  assert(#points > 0, "No points in path")
  assert(length > 0, "No path length")
  -- Since we are drawing, save the drawing of the path(s)
  assert(g_holo:save("/tmp/path_"..name..".pgm"))
  -- Add to the table of paths
  paths[name] = {
    points = points,
    length = length,
    ds = ds,
    closed = routes[name].closed
  }
  print(string.format("Route [%s] Length [%.2f meters] Points [%d]",
  name, length, #points))
end

-- For the roundabouts
-- local radius_roundabout1 = 1.5
-- local radius_roundabout2 = 1.75
-- do
--   local points, length = path.path_arc(
--     {2.5, 2.5}, radius_roundabout1, 0, 2 * math.pi, ds)
--   paths.roundabout1 = {
--     length = length,
--     ds = ds,
--     closed = true,
--     points = points
--   }
-- end
-- do
--   local points, length = path.path_arc(
--     {2.5, 2.5}, radius_roundabout2, 0, 2 * math.pi, ds)
--   paths.roundabout2 = {
--     length = length,
--     ds = ds,
--     closed = true,
--     points = points
--   }
-- end

local transitions = {}
-- All names are alphanumeric nameSEGMENT#lane#, where lane is positive or negative
local name_regex = "%l+%u+[+-]?%d+"
local lane_capture = string.format("^lane_(%s)$", name_regex)
local turn_capture = string.format("^turn_(%s)_(%s)$", name_regex, name_regex)
for name_l, path_l in pairs(paths) do
  -- print()
  local lane_name = name_l:match(lane_capture)
  local start_turn, end_turn = name_l:match(turn_capture)
  if lane_name then
    local list_t = {}
    -- print("lane_name", lane_name)
    if path_l.closed then table.insert(list_t, name_l) end
    -- print("turn_candidate", turn_candidate)
    for name_t, path_t in pairs(paths) do
      if name_t:match(turn_capture)==lane_name then
        -- print("name_t", name_t)
        local xa, ya = unpack(path_l.points[#path_l.points])
        local xb, yb = unpack(path_t.points[1])
        local dtransition = math.sqrt((xb-xa)^2 + (yb-ya)^2)
        local eps = 0.001
        assert(dtransition < eps, "Bad transition!")
        -- if dtransition > eps then
        --   print("Bad transition!")
        --   print("xa, xb", xa, xb)
        --   print("ya, yb", ya, yb)
        -- end
        table.insert(list_t, name_t)
      end
    end
    transitions[name_l] = list_t
  elseif end_turn then
    -- Add turn transition
    -- print("end_turn", end_turn)
    local lane_next = "lane_"..end_turn
    assert(type(paths[lane_next])=='table', "Bad turn endpoint")
    transitions[name_l] = lane_next
  else
    print("!! BAD NAME !!", name_l)
  end
end

-- Set the environment for displaying in-browser
local env = {
  viewBox = {g_holo.xmin, g_holo.ymin, g_holo.xmax - g_holo.xmin, g_holo.ymax - g_holo.ymin},
  observer = false,
  time_interval = dt,
  speed = vel_lane,
  paths = paths,
  transitions = transitions
}

--------------------------
-- Update the pure pursuit
local function cb_loop(t_us)
  local pose_rbt = veh_poses[id_robot]
  for k, v in pairs(env) do
    print(k, v)
  end
  -- for k, v in pairs(env.paths) do
  --   print(k, v)
  -- end
  for k, v in pairs(env.transitions) do
    print(string.format("%s:\t%s", k, type(v)=='table' and table.concat(v, ", ") or v))
  end
  assert(log_announce(log, env, "risk"))
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
  return 0
end
racecar.handle_shutdown(exit)

racecar.listen{
  channel_callbacks = cb_tbl,
  loop_rate = 1000, -- 1 Hz loop
  fn_loop = cb_loop,
  fn_debug = cb_debug
}
exit()
