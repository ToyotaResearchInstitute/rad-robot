#!/usr/bin/env luajit

local unpack = unpack or require'table'.unpack
local has_ff, ff = pcall(require, 'fileformats')

math.randomseed(123)
local rrt = require'rrt'


local tree = rrt.new{
  intervals = {{-1, 1}, {-1,1}}
}

tree:set_system{
  is_collision = function(self, s)
    if self:distState(s, {0,0}) < 0.25 then
      return true
    end
    return false
  end
}
-- Go from 0,0 to 1,1
tree:plan({-0.9,-0.9}, {0.9, 0.9})

-- Periodically, check to find the best trajectory
-- Should do this within a computational bound
if TEST_SIMPLE then

for i=1,10e3 do
  tree:iterate()
  if i>1e3 and tree.goal.parent then
    print(string.format("Iteration %d | Cost: %f",
                        i, tree.lowerBoundCost))
    break
  elseif i % 100 == 0 then
    print(string.format("Iteration %d | Cost: %f",
                        i, tree.goal.parent and tree.lowerBoundCost or 0/0))
  end
  -- for _, vertex in ipairs(tree) do print(vertex.id, unpack(vertex)) end
end

end

if tree.goal.parent then
  print("Node path from goal to start")
  local cur = tree.goal
  repeat
    print(cur.id, tree:distState(cur, {0,0}), unpack(cur))
    cur = cur.parent
  until not cur
end

-- os.exit()

local ffi = require'ffi'
ffi.cdef[[
typedef enum
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;
typedef struct
{
    /* the initial configuration */
    double qi[3];
    /* the lengths of the three segments */
    double param[3];
    /* model forward velocity / model angular velocity */
    double rho;
    /* the path type described */
    DubinsPathType type;
} DubinsPath;
int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho);
double dubins_path_length(DubinsPath* path);
int dubins_path_sample(DubinsPath* path, double t, double q[3]);
]]
local dubins = ffi.load"dubins"

local TEST_DUBINS = true
if TEST_DUBINS then
  local path = ffi.new"DubinsPath"
  local TURNING_RADIUS = 3 -- meters
  local q0 = ffi.new("double[3]", {0,0,0})
  local q1 = ffi.new("double[3]", {7,7,0})
  local ret = dubins.dubins_shortest_path(
    path, q0, q1, TURNING_RADIUS)
  print("Dubins return", ret)
  local length = dubins.dubins_path_length(path)
  print("Dubins length", length)
end
-- os.exit()

local TURNING_RADIUS = 3 -- meters
local DISCRETIZATION_STEP = 0.1
-- TODO: Set the dimensions for decent outputs
local interval_x = {-100, 100}
local interval_y = {-100, 100}
-- local interval_a = {-math.pi, math.pi}
local interval_a = {0, 2*math.pi}
local intervals = {interval_x, interval_y, interval_a}
local planner_car = rrt.new{
  intervals = intervals,
  DISCRETIZATION_STEP = DISCRETIZATION_STEP,
}:set_system{
  is_collision = function(self, s)
    for i, interval in ipairs(self.intervals) do
      if s[i] < interval[1] or s[i] > interval[2] then return true end
    end
    local obstacles = {{-40, 10, 15}, {-15, 15, 15}, {0, 0, 15}, {15, -15, 15}, {5, -25, 15},
                       {-90, -50, 11}, {-70, -50, 11}, {-50, -50, 11},
                      {0, -90, 11}, {0, -70, 11}, {0, -50, 11},
                      }
    for _, obs in ipairs(obstacles) do
      local dp = math.sqrt((s[1] - obs[1])^2 + (s[2] - obs[2])^2)
      if dp < (obs[3] or 1) then return true end
    end
    return false
  end,
  is_near_goal = function(self, state, goal)
    -- Returns false or distance to goal
    goal = goal or self.goal
    -- Check if Dubins reaches the goal collision free
    local reaches_goal, _, length, path = self:extend(state, goal)
    if reaches_goal then
      return length, path
    end
    -- Else, check if is close enough already
    local dp = math.sqrt((state[1] - goal[1])^2 + (state[2] - goal[2])^2)
    local close_position = dp < 0.25
    -- if #goal == 2 then
    --   return close_position and dp or false
    -- end
    local dth = state[3] - goal[3]
    if dth > math.pi then
      dth = dth - 2 * math.pi
    elseif dth < -math.pi then
      dth = dth + 2 * math.pi
    end
    local close_angle = math.abs(dth) < (10 * math.pi / 180)
    if close_position and close_angle then
      return dp
    end
    return false
  end,
  distance = function(self, from, to)
    -- Check if close enough to each other...?
    local path = ffi.new"DubinsPath"
    local q0 = ffi.new("double[3]", from)
    local q1 = ffi.new("double[3]", to)
    -- Should return the extra information
    local ret = dubins.dubins_shortest_path(
      path, q0, q1, TURNING_RADIUS)
    if ret~=0 then return false, "Bad Dubins path" end
    -- Give the length and the path
    local length = dubins.dubins_path_length(path)
    return length, path
  end,
  extend = function(self, state, target, length, path)
    -- Dubins path struct is our extra bits
    if not path then
      length, path = self:distState(state, target)
    end
    if not length then
      length = dubins.dubins_path_length(path)
    end

    -- Walk along the path and check for collisions
    local t = 0
    local step_size = self.DISCRETIZATION_STEP -- meters
    local q = ffi.new("double[3]")
    -- print("Length", length)
    while t < length do
      dubins.dubins_path_sample(path, t, q)
      local cur = {q[0], q[1], q[2]}
      local collides = self:is_collision(cur)
      -- print("Cur", collides, unpack(cur))
      -- Collision check this configuration
      if collides then
        return false, cur, t, path
      end
      t = t + step_size
    end
    return true, target, length, path
  end
}

print("Planning the car route!")

-- planner_car:plan({-85,-85, 0}, {85, 85, math.pi/2})
-- planner_car:plan({-80,-80, 0}, {75, 75, math.pi})
-- planner_car:plan({-75,-75, 0}, {75, 75, math.pi/2})
-- planner_car:plan({-75,-75, 0}, {75, 99, 0})
planner_car:plan({-75,-75, math.pi/2}, {75, 75, math.pi/2})
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
  i= i + 1
until dt > 5

local floor = require'math'.floor
if planner_car.goal.parent then
  print("Node path from goal to start")
  local x0, x1 = unpack(interval_x)
  local y0, y1 = unpack(interval_y)
  local m = floor((y1 - y0) / DISCRETIZATION_STEP)
  local n = floor((x1 - x0) / DISCRETIZATION_STEP)
  local n_cells = m * n
  local function coord2ind(x, y, a)
    return (a or 0) * n_cells + m * floor((x - x0) / DISCRETIZATION_STEP) + floor((y - y0) / DISCRETIZATION_STEP)
  end
  local path8 = has_ff and ffi.new('uint8_t[?]', n_cells)
  print("path8", path8, m, n)
  local cur = planner_car.goal
  local dist = 0
  local i = 0
  repeat
    local length = 0
    local path
    if cur.parent then
      length, path = planner_car:distState(cur.parent, cur)
    end
    dist = dist + length
    print(cur.id, length, unpack(cur))
    if path then
      -- Walk along the path and check for collisions
      local t = 0
      local step_size = DISCRETIZATION_STEP -- meters
      local q = ffi.new("double[3]")
      while t < length do
        dubins.dubins_path_sample(path, t, q)
        path8[coord2ind(q[0], q[1])] = 255
        t = t + step_size
      end
    end
    cur = cur.parent
    i = i + 1
  until not cur
  print("Total distance", dist)
  if path8 then
    print("Draw obstacles")
    local x= x0
    while x < x1 do
      local y = y0
      while y < y1 do
        -- print("Check", x, y)
        if planner_car:is_collision({x, y, 0}) then
          -- print("Obs!", x, y)
          path8[coord2ind(x, y)] = 127
        end
        y = y + DISCRETIZATION_STEP
      end
      x = x + DISCRETIZATION_STEP
    end

    print("Saving map...", path8)
    assert(ff.save_netpbm('/tmp/pathRRT.pgm', path8, m, n))
  end

end
