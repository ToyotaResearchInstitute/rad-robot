#!/usr/bin/env luajit

local min, max = require'math'.min, require'math'.max
local floor = require'math'.floor
local sformat = require'string'.format
local unpack = unpack or require'table'.unpack

local ffi = require'ffi'

ffi.cdef[[
int dijkstra_holonomic(double* cost_to_go, // output
                       double* costmap, //input
                       unsigned int m, unsigned int n,
                       int* p_goal, int* p_start,
                       int nNeighbors);
int dijkstra_nonholonomic(double* cost_to_go, // output
                          double* costmap, //input
                          unsigned int m, unsigned int n,
                          int* p_goal, int* p_start,
                          int nNeighbors);
]]
local dijkstra = ffi.load('dijkstra')

local sqrt = require'math'.sqrt
-- TODO: Share the neighbors with the C source
local neighbors4 = {
  {1, 0, 1.0},
  {0, 1, 1.0},
  {-1, 0, 1.0},
  {0, -1, 1.0},
}
local neighbors8 = {
  {1,0,1.0}, {1,1,sqrt(2)},
  {0,1,1.0}, {-1,1,sqrt(2)},
  {-1,0,1.0}, {-1,-1,sqrt(2)},
  {0,-1,1.0}, {1,-1,sqrt(2)},
}
local neighbors16 = {
  {1,0,1.0}, {2,1,sqrt(5)}, {1,1,sqrt(2)}, {1,2,sqrt(5)},
  {0,1,1.0}, {-1,2,sqrt(5)}, {-1,1,sqrt(2)}, {-2,1,sqrt(5)},
  {-1,0,1.0}, {-2,-1,sqrt(5)}, {-1,-1,sqrt(2)}, {-1,-2,sqrt(5)},
  {0,-1,1.0}, {1,-2,sqrt(5)}, {1,-1,sqrt(2)}, {2,-1,sqrt(5)},
}
local neighbor_options = {
  [4] = neighbors4,
  [8] = neighbors8,
  [16] = neighbors16,
}

local lib = {}

-- Trace a path from start to zero in the cost-to-go map
local function path_c2g_holonomic(self, cost_to_go, ijstart)
  local m, n = self.costmap.m, self.costmap.n
  local n_cells = m * n
  local neighbors = self.neighbors
  local path = {}
  local i0, j0 = unpack(ijstart, 1, 2)
  table.insert(path, {i0, j0})
  local nIter = 0
  repeat
    local min_v = cost_to_go[j0 * m + i0]
    local vbest = min_v
    local ibest, jbest
    --
    for _, offset in ipairs(neighbors) do
      local dj, di = unpack(offset, 1, 2)
      local i1, j1 = i0 + di, j0 + dj
      if i1>=0 and i1<m and j1>=0 and j1<n then
        local c2g = cost_to_go[j1 * m + i1]
        if c2g < vbest then
          vbest = c2g
          ibest, jbest = i1, j1
        end
      end
    end
    -- Check if reaching goal
    nIter = nIter + 1
    if nIter > n_cells then
      return false, "Long path"
    elseif vbest == min_v then
      return false, "Could not reach goal"
    end
    i0, j0 = ibest, jbest
    table.insert(path, {ibest, jbest})
    --
  until vbest == 0
  return path
end

local function path_c2g_nonholonomic(self, cost_to_go, ijastart)
  local m, n = self.costmap.m, self.costmap.n
  local n_cells = m * n
  local neighbors = self.neighbors
  local nNeighbors = #neighbors
  local shift_amount = math.floor(nNeighbors / 8)
  local path = {}
  local i0, j0, a0 = unpack(ijastart, 1, 3)
  table.insert(path, {i0, j0, a0})
  local nIter = 0
  repeat
    -- print("i0, j0, a0", i0, j0, a0)
    local ind0 = a0 * n_cells + j0 * m + i0
    local min_v = cost_to_go[ind0]
    --
    local vbest = min_v
    local ibest, jbest, abest, backbest
    -- Add forward cells
    local iangles = {}
    for ashift=-shift_amount,shift_amount do
      local a1 = (a0 + ashift + nNeighbors) % nNeighbors
      table.insert(iangles, a1)
    end
    local nForward = #iangles
    -- Add backwards cells
    for i=1, nForward do
      local a1 = iangles[i]
      local a2 = (a1 + nNeighbors/2) % nNeighbors
      table.insert(iangles, a2)
    end
    -- print("iangles", unpack(iangles))
    for ia, a1 in ipairs(iangles) do
      -- Move in the _current_ direction
      local offset = neighbors[a0+1]
      local dj, di = unpack(offset, 1, 2)
      local i1, j1
      -- Check if backwards
      local is_backwards = ia > nForward
      if is_backwards then
        -- Move backwards in the current direction
        i1, j1 = i0 - di, j0 - dj
        a1 = iangles[ia - nForward]
      else
        -- Move forwards in the current direction
        i1, j1 = i0 + di, j0 + dj
      end

      if i1>=0 and i1<m and j1>=0 and j1<n then
        local ind1 = a1 * n_cells + j1 * m + i1
        local c2g = cost_to_go[ind1]
        -- print("Angle", a1, c2g)
        if c2g < vbest then
          vbest = c2g
          ibest, jbest, abest = i1, j1, a1
          backbest = is_backwards
        end
      end
    end
    -- Check if reaching goal
    nIter = nIter + 1
    if nIter > n_cells then
      return false, "Long path"
    elseif vbest == min_v then
      return false, "Could not reach goal"
    end
    -- Add to path
    table.insert(path, {ibest, jbest, abest, backbest})
    -- Inspect the next cell
    i0, j0, a0 = ibest, jbest, abest
    --
  until vbest == 0
  return path
end

local function plan(self, start, goal)
  local neighbors = self.neighbors
  local nNeighbors = #neighbors
  local m, n = self.costmap.m, self.costmap.n
  local n_cells = m * n
  --
  local state_sz = #goal
  if #start~=state_sz then
    return false, "State size mismatch"
  end
  local tp_arr = sformat("int[%d]", state_sz)
  local indices_goal = ffi.new(tp_arr, {unpack(goal, 1, state_sz)})
  -- TODO: Support many starting indices, since the cost-to-go map is made
  local indices_start = ffi.new(tp_arr, {unpack(start, 1, state_sz)})
  -- Form the output memory for the cost-to-go map
  local cost_to_go
  if #goal==3 then
    cost_to_go = ffi.new('double[?]', n_cells * nNeighbors)
    dijkstra.dijkstra_nonholonomic(cost_to_go, self.costmap.grid,
                                   m, n,
                                   indices_goal, indices_start,
                                   nNeighbors)
    local path, err = path_c2g_nonholonomic(self, cost_to_go, start)
    table.insert(path, {unpack(goal, 1, state_sz)})
    self.cost_to_go = cost_to_go
    return path, err
  elseif #goal==2 then
    cost_to_go = ffi.new('double[?]', n_cells)
    dijkstra.dijkstra_holonomic(cost_to_go, self.costmap.grid,
                                m, n,
                                indices_goal, indices_start,
                                nNeighbors)
    local path, err = path_c2g_holonomic(self, cost_to_go, start)
    self.cost_to_go = cost_to_go
    return path, err
  else
    return false, "Bad state size"
  end
end

local function new(parameters)
  local obj= {
    plan = plan,
    costmap = parameters.costmap,
    holonomic = not parameters.nonholonomic,
    neighbors = neighbor_options[parameters.neighbors or 4] or neighbor_options[4]
  }
  return obj
end
lib.new = new

return lib
