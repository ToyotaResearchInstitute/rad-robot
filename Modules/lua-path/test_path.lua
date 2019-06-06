#!/usr/bin/env luajit
local name_test = arg[1]
local unpack = unpack or require'table'.unpack
local grid = require'grid'
local path = require'path'

local ds = 0.01
print("Route", name_test)

-- Holodeck Grid
-- local g_path = assert(grid.new{
--   scale = 0.01,
--   xmin = 0, xmax = 4.5,
--   ymin = -1, ymax = 6
-- })
-- Basketball court ~30m x ~15m
local g_path = assert(grid.new{
  scale = 0.01,
  -- xmin = 0, xmax = 30,
  xmin = 0, xmax = 15,
  -- ymin = -1, ymax = 14
  ymin = -1, ymax = 14
})

local route_offsets = {
  outer = {0,0},
  inner = {0,0},
  roundabout = {7.5, 2.5}
}

local routes = {}

-- Place the Area A
routes.outer = {
  {0.75, -0.75},
  {4, -0.75},
  {4, 5.75},
  {0.75, 5.75},
  turning_radius = 0.3,
  closed = true
}
--
routes.inner = {
  {1.25, -0.25, math.rad(90)},
  {1.25, 5.25, math.rad(0)},
  {3.5, 5.25, math.rad(270)},
  {3.5, -0.25, math.rad(180)},
  turning_radius = 0.3,
  closed = true
}

-- Smaller routes
routes.outerA = {
  {0.75, -0.75, math.rad(0)},
  {4, -0.75, math.rad(90)},
  {4, 2.5, math.rad(180)},
  {0.75, 2.5, math.rad(270)},
  turning_radius = 0.3,
  closed = true
}
routes.outerB = {
  {4, 5.75, math.rad(180)},
  {0.75, 5.75, math.rad(270)},
  {0.75, 2.5, math.rad(0)},
  {4, 2.5, math.rad(90)},
  turning_radius = 0.3,
  closed = true
}

local radius_roundabout = 1.5
routes.roundabout = {
  {radius_roundabout, -radius_roundabout},
  {radius_roundabout, radius_roundabout},
  {-radius_roundabout, radius_roundabout},
  {-radius_roundabout, -radius_roundabout},
  turning_radius = radius_roundabout,
  closed = true
}

routes.inner_to_roundabout = {
  {3.5, 4.0, math.rad(-90)},
  {3.5, 2.5, math.rad(-90)},
  {5.95, 2.5, math.rad(0)},
  -- {5.0, 2.0, math.rad(-90)},
  {6.05, 2.05},
  turning_radius = 0.3,
  closed = false
}

routes.roundabout_to_highway1 = {
  {9.0, 2.25},
  {9.2, 2.5},
  {11.0, 2.5},
  {10, 8},
  {0.75, 8},
  {0.75, 5},
  turning_radius = 1.0,
  closed = false
}

routes.roundabout_to_highway2 = {
  {7.75, 4},
  {7.5, 4.25},
  {7.5, 8.0},
  {7.0, 8},
  turning_radius = 1,
  closed = false
}

-- Place various components
-- TODO: Rotate?
for name, route in pairs(routes) do
  local cx, cy = 0, 0
  if name:find("outer")==1 or name:find("inner")==1 then
    cx, cy = 0, 0
  elseif name=="roundabout" then
    cx, cy = 7.5, 2.5
  end
  for i=1,#route do
    local x0, y0 = unpack(route[i])
    route[i] = {x0 + cx, y0 + cy}
  end
end

-- Iterate all of the routes
for name, route in pairs(routes) do
  if not name_test or name==name_test then
    print("Knots", name)
    for i, kn in ipairs(route) do print(i, unpack(kn)) end

    -- Control points are actually waypoints
    local waypoints_test = assert(path.generate_waypoints(route))
    -- Print the waypoints
    print("Waypoints", name)
    for i, wp in ipairs(waypoints_test) do print(i, unpack(wp)) end

    -- Generate the path
    local path_test, length = assert(path.path_from_waypoints(waypoints_test, {
      ds = ds,
      closed = route.closed
     }))
    -- Print the path
    print("Path", name)
    for i, p in ipairs(path_test) do print(i, unpack(p)) end

    -- Draw the path
    if name==name_test then
      g_path = assert(path.draw_grid(path_test))
      g_path:save(string.format("/tmp/path_%s.pgm", name))
      return
    else
      print("Adding path to the grid...")
      -- Only for continuous ones
      assert(g_path:path(path_test))
    end
  end
end
-- Save
print("Saving now")
g_path:save(string.format("/tmp/path.pgm"))
