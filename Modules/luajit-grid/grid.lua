local lib = {}

local ffi = require'ffi'

local has_ff, ff = pcall(require, 'fileformats')
local has_png, png = pcall(require, 'png')
local has_jpeg, jpeg = pcall(require, 'jpeg')
local c_gray
if has_jpeg then
  c_gray = jpeg.compressor('gray')
end

local abs = require'math'.abs
local atan2 = require'math'.atan2
local cos = require'math'.cos
local sin = require'math'.sin
local ceil = require'math'.ceil
local floor = require'math'.floor
local log = require'math'.log
local max = require'math'.max
local min = require'math'.min
local sqrt = require'math'.sqrt
local function sign(v)
  if v>0 then
    return 1
  elseif v<0 then
    return -1
  else
    return 0
  end
end
local unpack = unpack or require'table'.unpack

local function set_max(map, idx)
  map[idx] = 255
end

local function fill(self, value)
  local grid = self.grid
  for idx=0,self.n_cells-1 do
    grid[idx] = value
  end
  return self
end

local function point(self, xy, cb)
  if type(cb) ~='function' then cb = set_max end
  local grid = self.grid
  local xy2idx = self.xy2idx
  local idx = xy2idx(unpack(xy))
  if idx then cb(grid, idx) end
  return self
end

local function path(self, list, cb)
  if type(cb) ~='function' then cb = set_max end
  local grid = self.grid
  local xy2idx = self.xy2idx
  local idx1
  for ilist, xy in ipairs(list) do
    local idx = xy2idx(unpack(xy))
    if idx and idx~=idx1 then
      cb(grid, idx, ilist)
      idx1 = idx
    end
  end
  return self
end

-- callback cb is update_hit normally
local function bresenham(self, xy0, xy1, cb)
  -- Remember - y and i, x and j
  local grid = self.grid
  local ij2idx = self.ij2idx
  if type(cb) ~='function' then cb = set_max end
  -- Rastered
  local i0, j0 = self.xy2ij(unpack(xy0))
  local i1, j1 = self.xy2ij(unpack(xy1))
  -- NOTE:
  local di, dj = i1 - i0, j1 - j0
  local di_sign, dj_sign = sign(di), sign(dj)
  -- Check for simplifications
  if di_sign==0 and dj_sign==0 then
    local idx = ij2idx(i0, j0)
    if idx then cb(grid, idx, i0, j0) end
  elseif i0==i1 then
    for j=j0,j1,dj_sign do
      local idx = ij2idx(i0, j)
      if idx then cb(grid, idx, i0, j) end
    end
  elseif j0==j1 then
    for i=i0,i1,di_sign do
      local idx = ij2idx(i, j0)
      if idx then cb(grid, idx, i, j0) end
    end
  -- Check the slope so that the direction is correct
  elseif abs(dj) >= abs(di) then
    local deltaerr = abs(di / dj)
    local i = i0
    local err = 0
    for j = j0, j1, dj_sign do
      local idx = ij2idx(i, j)
      if idx then cb(grid, idx, i, j) end
      err = err + deltaerr
      if err >= 0.5 then
        err = err - 1
        i = i + di_sign
      end
    end
  else
    local deltaerr = abs(dj / di)
    local j = j0
    local err = 0
    for i = i0, i1, di_sign do
      local idx = ij2idx(i, j)
      if idx then cb(grid, idx, i, j) end
      err = err + deltaerr
      if err >= 0.5 then
        err = err - 1
        j = j + dj_sign
      end
    end
  end
  return self
end

-- Draw a filled circle
local function circle(self, pc, rc, cb)
  -- pc: Center point
  -- rc: Radius from center
  if type(cb) ~='function' then cb = set_max end
  local grid = self.grid
  local scale = self.scale
  local ij2idx = self.ij2idx
  local xy2ij = self.xy2ij
  local xc, yc = unpack(pc, 1, 2)
  -- Compute the bounding box within which to draw the circle
  -- Later idx calculations are safe due to these min/max calls
  local imin, jmin = xy2ij(xc - rc, yc - rc)
  local imax, jmax = xy2ij(xc + rc, yc + rc)
  imin = max(0, min(imin, self.m - 1))
  imax = max(0, min(imax, self.m - 1))
  jmin = max(0, min(jmin, self.n - 1))
  jmax = max(0, min(jmax, self.n - 1))
  local ic, jc = xy2ij(xc, yc)
  -- Loop within the bounding box
  for i=imin, imax do
    for j=jmin, jmax do
      local dx, dy = scale * (i - ic), scale * (j - jc)
      local d = sqrt(dx * dx + dy * dy)
      if d <= rc then
        local idx = ij2idx(i, j)
        cb(grid, idx, d, dx, dy)
      end
    end
  end
  return self
end

local function arc(self, pc, rc, a1, a2, cb)
  -- pc: Center point
  -- rc: Radius from center
  -- a1: start angle (inclusive)
  -- a2: stop angle (inclusive)
  if type(cb) ~='function' then cb = set_max end
  local grid = self.grid
  local xy2ij, ij2idx = self.xy2ij, self.ij2idx
  local xc, yc = unpack(pc, 1, 2)
  local idx1
  local dir = a2 > a1 and 1 or -1
  local ang_resolution = dir * atan2(self.scale, rc)
  for angle=a1, a2, ang_resolution do
    local c, s = cos(angle), sin(angle)
    local dx, dy = c * rc, s * rc
    local px, py = xc + dx, yc + dy
    local ip, jp = xy2ij(px, py)
    local idx = ij2idx(ip, jp)
    if idx and idx~=idx1 then
      cb(grid, idx, dx, dy)
      idx1 = idx
    end
  end
  return self
end

local function voronoi(self, centers)
  local grid = self.grid
  local idx2xy = self.idx2xy
  for idx=0,self.n_cells-1 do
    local x, y = idx2xy(idx)
    local id = 0
    local d = math.huge
    for idc, c in ipairs(centers) do
      local xc, yc = unpack(c)
      local dx, dy = x - xc, y - yc
      local dc = sqrt(dx * dx + dy * dy)
      -- Distance to the obstacle is the distance to the center,
      -- less the radius of the circular obstacle
      if dc < d then
        d = dc
        id = idc
      end
    end
    grid[idx] = id
  end
  return self
end

-- https://www.cs.cmu.edu/~motionplanning/lecture/Chap4-Potential-Field_howie.pdf
local gain_attractive = 5
local gain_repulsive = 1e2
local function potential_field(self, goal, obstacles)
  local grid = self.grid
  local idx2xy = self.idx2xy
  local dx, dy, d_sq
  for idx=0,self.n_cells-1 do
    local x, y = idx2xy(idx)
    -- Attract towards goal
    local xg, yg = unpack(goal)
    dx, dy = x - xg, y - yg
    d_sq = dx * dx + dy * dy
    local attraction = 0.5 * gain_attractive * sqrt(d_sq)
    -- Set the potential as the attraction, as a base case
    local potential = -log(attraction)
    -- Repulsive is from the closest obstacle
    --[[
    local repulsion
    local d_obs = math.huge
    for _, obs in ipairs(obstacles) do
      local io, jo, ro = unpack(obs)
      local d_center = math.sqrt((i - io)^2 + (j - jo)^2)
      -- Distance to the obstacle is the distance to the center,
      -- less the radius of the circular obstacle
      d_obs = min(d_obs, d_center - ro)
    end
    if d_obs <= 0 then
      repulsion = math.huge
      grid[idx] = repulsion
    else
      repulsion = gain_repulsive * (0.5 / d_obs ^ 2)
      local v = math.log(repulsion) - math.log(attraction)
      print(v)
      grid[idx] = v
    end
    --]]
    for _, obs in ipairs(obstacles) do
      local xo, yo, ro = unpack(obs)
      dx, dy = x - xo, y - yo
      d_sq = dx * dx + dy * dy
      local d_center = sqrt(d_sq)
      -- Distance to the obstacle is the distance to the center,
      -- less the radius of the circular obstacle
      local d_obs = d_center - ro
      if d_obs <= 0 then
        potential = math.huge
        break
      end
      local repulsion = gain_repulsive * (0.5 / d_obs ^ 2)
      potential = potential + math.log(repulsion)
    end
    grid[idx] = potential
  end
  return self
end

local function set_indexing(self)
  -- Utility functions for accessing the map
  -- Positive y maps to positive i
  -- Positive x maps to positive j
  -- Thus, x and j are linked and y and i are mapped
  -- So, down an image is forward x.
  local m, n = self.m, self.n
  local xmin, ymin = self.xmin, self.ymin
  local meters_per_cell = self.scale
  local cells_per_meter = self.scale_inv
  local function ij2idx(i, j)
    if i<0 or i>=m or j<0 or j>=n then
      return false
    end
    return j * m + i
  end
  self.ij2idx = ij2idx
  local function x2j(x)
    return floor(cells_per_meter * (x - xmin))
  end
  self.x2j = x2j
  local function y2i(y)
    return floor(cells_per_meter * (y - ymin))
  end
  self.y2i = y2i
  local function xy2ij(x, y)
    return y2i(y), x2j(x)
  end
  self.xy2ij = xy2ij
  function self.xy2idx(x, y)
    return ij2idx(xy2ij(x, y))
  end
  local function idx2ij(ind)
    local i = ind % m
    local j = floor(ind / m)
    -- local j = (ind - i) / m
    return i, j
  end
  self.idx2ij = idx2ij
  local function j2x(j)
    return meters_per_cell * j + xmin
  end
  self.j2x = j2x
  local function i2y(i)
    return meters_per_cell * i + ymin
  end
  self.i2y = i2y
  local function ij2xy(i, j)
    return j2x(j), i2y(i)
  end
  self.ij2xy = ij2xy
  function self.idx2xy(ind)
    return ij2xy(idx2ij(ind))
  end
  return self
end

local function save(self, fname, options)
  local grid = self.grid
  local grid_to_save = grid
  if type(options)~='table' then options = {} end

  local outmax = 255
  if self.datatype~='uint8_t' then
    local vmin, vmax = math.huge, -math.huge
    for i=0,self.n_cells-1 do
      local v = grid[i]
      if abs(v) ~= math.huge then
        vmin = min(v, vmin)
        vmax = max(v, vmax)
      end
    end
    -- Must rescale
    grid_to_save = ffi.new("uint8_t[?]", self.n_cells)
    local scale_factor = outmax / (vmax - vmin)
    for i=0,self.n_cells-1 do
      local v = grid[i]
      if v == math.huge then
        grid_to_save[i] = outmax
      elseif v == -math.huge then
        grid_to_save[i] = 0
      else
        grid_to_save[i] = scale_factor * (v - vmin)
      end
    end
  elseif options.use_max then
    local vmin, vmax = 255, 0
    for i=0,self.n_cells-1 do
      local v = grid[i]
      vmin = min(v, vmin)
      vmax = max(v, vmax)
    end
    -- Likely not needed, but whatever
    outmax = min(255, vmax)
  end
  -- Try to save
  if has_ff and fname:match"%.pgm$" then
    return ff.save_netpbm(
      fname,
      grid_to_save,
      self.m, self.n,
      {
        scale = self.scale,
        xmin = self.xmin,
        ymin = self.ymin,
        xmax = self.xmax,
        ymax = self.ymax,
      },
      options.use_max and outmax)
  elseif has_png and fname:match"%.png$" then
    local ptr = tonumber(ffi.cast('intptr_t', ffi.cast('void *', grid_to_save)))
    local str = png.compress(ptr, self.n_cells, self.m, self.n, 1)
    if fname=='png' then
      return str
    else
      return io.open(fname, "w"):write(str):close()
    end
  elseif has_jpeg and fname:match"%.jp[e]?g$" then
    local ptr = tonumber(ffi.cast('intptr_t', ffi.cast('void *', grid_to_save)))
    local str = c_gray:compress(ptr, self.n_cells, self.m, self.n)
    if fname=='jpeg' or fname=='jpg' then
      return str
    else
      return io.open(fname, "w"):write(str):close()
    end
  else
    return false, "This extension not supported"
  end
end

-- Make a new grid
function lib.new(params)
  params = params or {}
  local datatype = params.datatype or "uint8_t"
  local obj = {}
  if has_ff and type(params.fname)=='string' then
    local img, dimensions, comments = ff.load_netpbm(params.fname)
    -- Set the grid
    obj.grid = img
    -- m is the width, while n is the height
    obj.m, obj.n = unpack(dimensions, 1, 2)
    obj.n_cells = obj.m * obj.n
    -- TODO: Ensure that the number of channels is 1...
    for _, str in ipairs(comments) do
      local name, val = str:match"# (%S+): (%S+)"
      obj[name] = tonumber(val) or val
    end
  end

  -- Set the boundaries of the map
  if type(obj.xmin)~='number' then
    obj.xmin = tonumber(params.xmin) or -1
  end
  if type(obj.ymin)~='number' then
    obj.ymin = tonumber(params.ymin) or -1
  end
  if type(obj.xmax)~='number' then
    obj.xmax = tonumber(params.xmax) or 1
  end
  if type(obj.ymax)~='number' then
    obj.ymax = tonumber(params.ymax) or 1
  end
  -- TODO: Check that max>min

  -- Set the scale
  if type(obj.scale)~='number' then
    -- Default of 10 cm per cell
    -- scale: units of "meters per cell"
    obj.scale = tonumber(params.scale) or 0.1
  end
  -- scale_inv: units of "cells_per_meters"
  obj.scale_inv = 1 / obj.scale

  -- Establish the grid memory
  if not obj.grid then
    -- NOTE: Working
    obj.n = floor(obj.scale_inv * (obj.xmax - obj.xmin)) --n/j/x
    obj.m = floor(obj.scale_inv * (obj.ymax - obj.ymin)) -- m/i/y
    -- NOTE: Try this:
    obj.n = ceil(obj.scale_inv * (obj.xmax - obj.xmin)) --n/j/x
    obj.m = ceil(obj.scale_inv * (obj.ymax - obj.ymin)) -- m/i/y
    --
    obj.n_cells = obj.m * obj.n
    if type(params.grid)=='cdata' then
      obj.grid = params.grid
    else
      if obj.n_cells >= math.huge then
        return false, "Too many cells"
      elseif obj.n_cells <= 0 then
        return false, "No cells"
      end
      obj.grid = ffi.new(datatype.."[?]", obj.n_cells)
    end
  end
  -- Pointer to the grid memory as a Lua number for C functions
  obj.ptr = tonumber(ffi.cast('intptr_t', ffi.cast('void *', obj.grid)))
  obj.datatype = datatype
  obj.datatype_sz = ffi.sizeof(datatype)

  -- Set the indexing methods
  set_indexing(obj)

  -- Add functions
  obj.arc = arc
  obj.bresenham = bresenham
  obj.circle = circle
  obj.fill = fill
  obj.path = path
  obj.point = point
  obj.potential_field = potential_field
  obj.voronoi = voronoi
  obj.save = save

  return obj
end

return lib