#!/usr/bin/env luajit

local path = require'path'
local unpack = unpack or require'table'.unpack

local ds = 0.01
local name_test = arg[1] or 'outer'
print("Route", name_test)

local routes = {}

-- Route specified by knots
routes.outer = {
  {0.75, -0.75},
  {4, -0.75},
  {4, 5.75},
  {0.75, 5.75},
  turning_radius = 0.3,
  closed = true
}

local radius_roundabout1 = 1.5
local radius_roundabout2 = 2.5
routes.roundabout1 = {
  {2.5 + radius_roundabout1, 2.5 - radius_roundabout1},
  {2.5 + radius_roundabout1, 2.5 + radius_roundabout1},
  {2.5 - radius_roundabout1, 2.5 + radius_roundabout1},
  {2.5 - radius_roundabout1, 2.5 - radius_roundabout1},
  turning_radius = radius_roundabout1,
  closed = true
}

routes.exit = {
  {2.5 + radius_roundabout1, 2.5 - radius_roundabout1},
  {2.5 + radius_roundabout1, 2.5 + radius_roundabout1},
  {2.5 - radius_roundabout1, 2.5 + radius_roundabout1},
  {2.5 - radius_roundabout1, 2.5 - radius_roundabout1},
  turning_radius = radius_roundabout1,
  closed = false
}

local route_test = routes[name_test]

print("Knots")
for i, kn in ipairs(route_test) do print(i, unpack(kn)) end

-- Control points are actually waypoints
local waypoints_test = assert(path.generate_waypoints(route_test))

print("Waypoints")
for i, wp in ipairs(waypoints_test) do print(i, unpack(wp)) end

local path_test, length = assert(path.path_from_waypoints(waypoints_test, {
  ds = ds,
  closed = route_test.closed
 }))

print("Path")
for i, p in ipairs(path_test) do print(i, unpack(p)) end

local g = assert(path.draw_grid(path_test))
g:save(string.format("/tmp/path_%s.pgm", name_test))
