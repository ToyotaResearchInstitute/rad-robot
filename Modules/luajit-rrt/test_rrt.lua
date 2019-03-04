#!/usr/bin/env luajit

local unpack = unpack or require'table'.unpack
-- math.randomseed(123)
local rrt = require'rrt'
local grid = require'grid'

-- Go from 0,0 to 1,1
local interval_x = {-1, 1}
local interval_y = {-1, 1}
local planner = rrt.new({
  intervals = {interval_x, interval_y}
}):set_system{
  is_collision = function(self, s)
    return self:distState(s, {0,0}) < 0.25
  end
}
-- Go from 0,0 to 1,1
local start_xy = {-0.9, -0.9}
local goal_xy = {0.9, 0.9}
planner:plan(start_xy, goal_xy)

-- Periodically, check to find the best trajectory
local i = 0
local t0 = os.time()
repeat
  local dt = os.difftime(os.time(), t0)
  planner:iterate()
  if i>1e3 and planner.goal.parent then
    print(string.format("Iteration %d | Cost: %f",
                        i, planner.lowerBoundCost))
    break
  elseif i % 100 == 0 then
    print(string.format("Iteration %d | Cost: %f",
                        i, planner.goal.parent and planner.lowerBoundCost or 0/0))
  end
  -- for _, vertex in ipairs(tree) do print(vertex.id, unpack(vertex)) end
until dt > 2

local path_xy, path_length = assert(planner:trace())
print("Path", path_length)

local grid_params = {
  scale = planner.DISCRETIZATION_STEP,
  xmin = interval_x[1], xmax = interval_x[2],
  ymin = interval_y[1], ymax = interval_y[2],
  datatype = 'double'
}
local costmap = grid.new(grid_params)
local function color_path(map, idx) map[idx] = 127 end
costmap:path(path_xy, color_path)
assert(costmap:save"/tmp/costmapRRT_path_default.pgm")


-- Test the dubins model for the car

local interval_x = {-10, 10}
local interval_y = {-10, 10}
local interval_a = {0, 2*math.pi}
local intervals = {interval_x, interval_y, interval_a}

local DISCRETIZATION_STEP = 0.1

local CIRCLE_RADIUS = 2
local circular_obstacles = {
  {2, -2, CIRCLE_RADIUS},
  {0, 0, CIRCLE_RADIUS},
  {-2, 2, CIRCLE_RADIUS},
}

local grid_params = {
  scale = DISCRETIZATION_STEP,
  xmin = interval_x[1], xmax = interval_x[2],
  ymin = interval_y[1], ymax = interval_y[2],
  datatype = 'double'
}
local costmap = require'grid'.new(grid_params)
local function set_obs(map, idx)
  map[idx] = 255
end
for _,c in ipairs(circular_obstacles) do
  local xc, yc, rc = unpack(c)
  costmap:circle({xc, yc}, rc, set_obs)
end

assert(costmap:save"/tmp/costmapRRT.pgm")

print("Planning the car route!")
local planner_car = assert(rrt.new{
  intervals = intervals,
  DISCRETIZATION_STEP = DISCRETIZATION_STEP,
})
local sys_dubins = assert(rrt.systems.dubins({
  TURNING_RADIUS = 3,
  circular_obstacles = circular_obstacles
}))
assert(planner_car:set_system(sys_dubins))

local start = {-7.5,-7.5, 0}
local goal = {7.5, 7.5, 0}
assert(planner_car:plan(start, goal))
-- Periodically, check to find the best trajectory
-- Should do this within a computational bound
local i = 0
local t0 = os.time()
repeat
  local dt = os.difftime(os.time(), t0)
  local ret, err = planner_car:iterate()
  if not ret then print("Bad iteration", err) end
  if i>1e3 and planner_car.goal.parent then
    print(string.format("%d | Iteration %d | Cost: %f",
                        dt, i, planner_car.lowerBoundCost))
    -- break
  elseif i % 100 == 0 then
    print(string.format("%d | Iteration %d | Cost: %f",
                        dt, i, planner_car.goal.parent and planner_car.lowerBoundCost or 0/0))
  end
  i = i + 1
until dt > 5

local path_xy, path_length = assert(planner_car:trace())
print("path_xy", path_xy)
for i, v in ipairs(path_xy) do
  print(i, v)
end

print("Path length", path_length)
local function color_path(map, idx, i) map[idx] = 127 end
costmap:path(path_xy, color_path)
assert(costmap:save"/tmp/costmapRRT_path.pgm")
