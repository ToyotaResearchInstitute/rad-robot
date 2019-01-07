#!/usr/bin/env luajit

local unpack = unpack or require'table'.unpack
local min, max = require'math'.min, require'math'.max
local floor = require'math'.floor

-- local interval_x = {0, 60}
-- local interval_y = {0, 80}

local interval_x = {-10, 10}
local interval_y = {-10, 10}
local interval_scale = 0.05
local grid_params = {
  scale = interval_scale,
  xmin = interval_x[1], xmax = interval_x[2],
  ymin = interval_y[1], ymax = interval_y[2],
  datatype = 'double'
}
local costmap = require'grid'.new(grid_params)

for ind=0, costmap.n_cells-1 do
  costmap.grid[ind] = 1
end

local maxvalLine = 1e5--8192 -- 8192 -- 4096 -- 2048 -- 1024 -- 256
local function set_max(map, idx)
  map[idx] = maxvalLine
end
costmap:bresenham(
  {-7, 7},
  {7, -7},
  set_max
):bresenham(
  {-7+costmap.scale, 7},
  {7+costmap.scale, -7},
  set_max
):bresenham(
  {-7-costmap.scale, 7},
  {7-costmap.scale, -7},
  set_max
)

local CIRCLE_RADIUS = 2
local maxvalCircle = maxvalLine / 3
local function set_circle(map, idx, r)
  -- map[idx] = maxval * (r==0 and 1 or math.sqrt(CIRCLE_RADIUS / r))
  map[idx] = maxvalCircle
end
local circles = {
  {2, -2, CIRCLE_RADIUS},
  {0, 0, CIRCLE_RADIUS},
  {-2, 2, CIRCLE_RADIUS},
}
for _,c in ipairs(circles) do
  local xc, yc, rc = unpack(c)
  costmap:circle(xc, yc, rc, set_circle)
end
assert(costmap:save"/tmp/costmapD")

local nNeighbors = 16
local RADIANS_PER_NEIGHBOR = 2*math.pi / nNeighbors

local planner = require'dijkstra'.new{
  costmap = costmap,
  neighbors = nNeighbors
}

local start_xy = {-9, -9}
start_xy[3] = math.pi/2
local start_ij = {costmap.xy2ij(unpack(start_xy))}
start_ij[3] = (start_xy[3] - start_xy[3] % RADIANS_PER_NEIGHBOR) / RADIANS_PER_NEIGHBOR

local goal_xy = {9, 9}
goal_xy[3] = 0
local goal_ij = {costmap.xy2ij(unpack(goal_xy))}
goal_ij[3] = (goal_xy[3] - goal_xy[3] % RADIANS_PER_NEIGHBOR) / RADIANS_PER_NEIGHBOR

print("Start_xy", table.concat(start_xy, ','))
print("Goal_xy", table.concat(goal_xy, ','))

print("Start_ij", table.concat(start_ij, ','))
print("Goal_ij", table.concat(goal_ij, ','))

--
local path_ij, err = planner:plan(start_ij, goal_ij)
if planner.cost_to_go then
  grid_params.grid = planner.cost_to_go
  local c2g = require'grid'.new(grid_params)
  assert(c2g:save"/tmp/c2gD")
end
assert(path_ij, err)

print("Path", type(path_ij))
local path_xy = {}
for i, v in ipairs(path_ij) do
  local xy = {costmap.ij2xy(unpack(v, 1, 2))}
  if v[3] then
    xy[3] = v[3] * (2 * math.pi)/ nNeighbors
  end
  table.insert(path_xy, xy)
  path_xy[i] = xy
end

print("saving")
local function color_path(map, idx, i)
  map[idx] = path_ij[i][4] and (maxvalLine/4) or (maxvalLine/2)
end
costmap:path(path_xy, color_path)
assert(costmap:save"/tmp/costmapD_path")
