#!/usr/bin/env luajit

local unpack = unpack or require'table'.unpack
local min, max = require'math'.min, require'math'.max
local floor = require'math'.floor
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

local has_ff, ff = pcall(require, 'fileformats')

local USE_NON_HOLONOMIC = true
local USE_BACKWARDS = true
-- local m, n = 320, 240
-- j is x, i is y
-- local m, n = 160, 120
local m, n = 60, 80
-- local m, n = 40, 30
-- local m, n = 20, 15

local n_cells = m * n
local nNeighbors = 8

local istart, jstart = 5, 5
local astart = 6
local igoal, jgoal = m-5, n-5
local agoal = 0

print("Goal", igoal, jgoal, agoal)
print("Start", istart, jstart, astart)

--[[
local function coord2ind(i, j, a)
  return (a and (a * n_cells) or 0) + i * n + j
end
local function ind2coord(ind)
  local i = floor(ind / n)
  local j = ind % n
  return i, j
end
local function ind2coordA(ind)
  local a0 = floor(ind / n_cells)
  local ij0 = ind0 - a0 * n_cells
  local j0 = ij0 / m
  local i0 = ij0 % m
  return i, j, a0
end
--]]

local function coord2ind(i, j, a)
  return (a or 0) * n_cells + j * m + i
end
local function ind2coord(ind)
  local j = floor(ind / m)
  local i = ind % m
  return i, j
end

-- Initialize the cost map
local costmap = ffi.new('double[?]', n_cells)
local cost_max = -math.huge
local cost_min = math.huge

for ind=0, n_cells-1 do
  -- local i,j = ind2coord(ind)
  -- local cost1 = 10 / (1 + math.abs(i - igoal, 2) + math.abs(j - jgoal, 2))
  -- local cost2 = 10 / (1 + math.sqrt( (i - istart)^2 + (j - jstart)^2) )
  -- costmap[ind] = max(cost1, cost2) + 1
  costmap[ind] = 1
end

local CIRCLE_RADIUS = 5
local circles = {
  {m - m/4, n/4, CIRCLE_RADIUS},
  {m/2, n/2, CIRCLE_RADIUS},
  {m/4, n/4, CIRCLE_RADIUS},
}
for _,c in ipairs(circles) do
  local ic, jc, rc = unpack(c)
  for ind=0, n_cells-1 do
    local i,j = ind2coord(ind)
    local dp = math.sqrt((i - ic)^2 + (j - jc)^2)
    if dp <= rc then costmap[ind] = 255 end
  end
end

-- math.randomseed(1234)
-- math.randomseed(os.time())
-- local n_circle = 20
-- local r_circle = min(m, n)/ (n_circle / 2)
-- for _=1,n_circle do
--   local i, j = math.random(m), math.random(n)
--   for di=-r_circle,r_circle do
--     for dj=-r_circle,r_circle do
--       local ii, jj = i + di, j+dj
--       if ii>=0 and ii<m and jj>=0 and jj<n then
--         local ind = coord2ind(ii, jj)
--         costmap[ind] = costmap[ind] + 50/(1 + 0.25*math.sqrt(di*di+dj*dj))
--       end
--     end
--   end
-- end

for ind=0, n_cells-1 do
  local cost = costmap[ind]
  cost_max = max(cost, cost_max)
  cost_min = min(cost, cost_min)
end

print("cost_max", cost_max)
print("cost_min", cost_min)

--[[
for i=0,m-1 do
  for j=0,n-1 do
    local cost = costmap[coord2ind(i, j)]
    io.write(cost==math.huge and "inf " or string.format("%d ", cost))
  end
  io.write('\n')
end
--]]

assert(cost_max > 0, "Bad max cost")
assert(cost_min > 0, "Bad min cost")

if has_ff then
  local costmap8 = ffi.new('uint8_t[?]', n_cells)
  for i=0, n_cells-1 do costmap8[i] = costmap[i] / cost_max * 255 end
  local fname_costmap = '/tmp/costmap.pgm'
  print("fname_costmap", fname_costmap)
  os.remove(fname_costmap)
  assert(ff.save_netpbm(fname_costmap, costmap8, m, n))
else
  print("No ff")
end

-- Output the cost to go
local cost_to_go
if USE_NON_HOLONOMIC then
  cost_to_go = ffi.new('double[?]', n_cells*nNeighbors)
  dijkstra.dijkstra_nonholonomic(cost_to_go, costmap,
                                 m, n,
                                 ffi.new('int[3]', {igoal, jgoal, agoal}),
                                 ffi.new('int[3]', {istart, jstart, astart}),
                                 nNeighbors)
else
  cost_to_go = ffi.new('double[?]', n_cells)
  agoal = 0
  astart = 0
  dijkstra.dijkstra_holonomic(cost_to_go, costmap,
                              m, n,
                              ffi.new('int[3]', {igoal, jgoal, agoal}),
                              ffi.new('int[3]', {istart, jstart, astart}),
                              nNeighbors)
end
-- os.exit()

local c2g_max = -math.huge
local c2g_min = math.huge
for ind=0, (USE_NON_HOLONOMIC and nNeighbors or 1) * n_cells-1 do
  local c2g = cost_to_go[ind]
  if c2g==0 then
    print("zero @", floor(ind/n_cells), ind2coord(ind%(USE_NON_HOLONOMIC and nNeighbors or 1)))
  end
  if c2g~=math.huge then
    c2g_max = max(c2g_max, c2g)
    c2g_min = min(c2g_min, c2g)
  end
end
print("c2g_max", c2g_max)
print("c2g_min", c2g_min)

--[[
for i=0,m-1 do
  for j=0,n-1 do
    local c2g = cost_to_go[coord2ind(i, j)]
    io.write(c2g==math.huge and "inf " or string.format("%d ", c2g))
  end
  io.write('\n')
end
--]]

assert(c2g_max > 0, "Bad max cost_to_go")
assert(c2g_min >= 0, "Bad min cost_to_go")
-- os.exit()

if has_ff then
  -- local n_cells1 = n_cells * (USE_NON_HOLONOMIC and nNeighbors or 1)
  local cost_to_go8 = ffi.new('uint8_t[?]', n_cells)
  for i=0, n_cells-1 do
    local c2g = cost_to_go[i]
    cost_to_go8[i] = (c2g==math.huge and 1 or c2g / c2g_max) * 255
  end
  assert(ff.save_netpbm('/tmp/cost_to_go.pgm', cost_to_go8, m, n))
end





local sqrt = require'math'.sqrt

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

local neighbors
if nNeighbors==8 then
  neighbors = neighbors8
elseif nNeighbors==16 then
  neighbors = neighbors16
else
  neighbors = neighbors4
end

io.stderr:write("Path gen\n")

local path = {}
table.insert(path, {istart, jstart, astart})

local i0, j0 = istart, jstart
local a0 = astart
local k

-- os.exit()
local shift_amount = nNeighbors<16 and 1 or 2
if not USE_NON_HOLONOMIC then
  shift_amount = 0
end
local nIter = 0
while nIter <= n_cells do
  nIter = nIter + 1
  -- print("* Coord", i0,j0,a0)
  local ind0 = coord2ind(i0, j0, USE_NON_HOLONOMIC and a0 or 0)
  -- io.stderr:write(string.format("Reconstruct %d, %d @ %d\n",
  --   i0, j0, a0))
  -- io.stderr:write(string.format("Reconstruct %d / %d\n",
  --   ind0, nNeighbors * n_cells))
  -- Iterate over neighbor items
  local min_v = cost_to_go[ind0]

  --
  local ibest, jbest, abest = i0, j0, a0
  local indbest = -1
  local vbest = min_v
  local iangles = {}
  if USE_NON_HOLONOMIC then
    for ashift=-shift_amount,shift_amount do
      local a1 = (a0 + ashift + nNeighbors) % nNeighbors
      table.insert(iangles, a1)
      if USE_BACKWARDS then
        local a2 = (a1 + nNeighbors/2) % nNeighbors
        table.insert(iangles, a2)
      end
    end
  else
    for a1=0, nNeighbors-1 do table.insert(iangles, a1) end
  end
  for ia, a1 in ipairs(iangles) do
    -- io.stderr:write("a1: ", a1, "\n")
    -- local dj, di = unpack(neighbors[a1+1], 1, 2)
    local dj, di = unpack(neighbors[a0+1], 1, 2)
    local is_backwards = USE_BACKWARDS and (ia % 2 == 0)
    if is_backwards then
      -- print("check back...")
      dj = dj * -1
      di = di * -1
      a1 = iangles[ia - 1]
    end
    local i1 = i0 + di
    local j1 = j0 + dj
    -- print((is_backwards and "~" or "=").." Coord",i1,j1,a1)
    if i1>=0 and i1<m and j1>=0 and j1<n then
      local ind1 = coord2ind(i1, j1, USE_NON_HOLONOMIC and a1 or 0)
      -- io.stderr:write("Checking: ", ind1, " of ", n_cells, "\n")
      local c2g = cost_to_go[ind1]
      -- print("= Cost", vbest, c2g)
      print("Angle", a1, c2g)
      if c2g < vbest then
        indbest = ia
        vbest = c2g
        ibest = i1
        jbest = j1
        abest = a1
        if is_backwards then
          print("Choosing Backwards!", indbest)
          -- abest = iangles[ia - 1]
        end
      end
    end
  end
  if min_v == vbest then
    print("Nobody new", min_v)
    break
  end
  i0 = ibest
  j0 = jbest
  a0 = abest

  table.insert(path, {i0, j0, a0, indbest})
  if vbest == 0 then
    print("Done!")
    break
  end
end
io.stderr:write("nIter: ", nIter, "\n")
-- table.insert(path, {igoal, jgoal, agoal})
print("Path", type(path))
for i, v in ipairs(path) do
  print("Path "..i, unpack(v))
end

print("saving")
if has_ff then
  local path8 = ffi.new('uint8_t[?]', n_cells)
  for i, v in ipairs(path) do
    local ibest, jbest, abest, indbest = unpack(v)
    local ind = coord2ind(ibest, jbest)
    if indbest and USE_BACKWARDS and indbest % 2 == 0 then
      print("Backwards!", indbest)
      path8[ind] = 127
    else
      path8[ind] = 255
    end
  end
  assert(ff.save_netpbm(USE_NON_HOLONOMIC and '/tmp/path1.pgm' or '/tmp/path.pgm', path8, m, n))
end
