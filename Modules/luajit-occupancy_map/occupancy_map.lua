-- (c) 2013 Stephen McGill
-- (c) 2018 Stephen McGill
-- Maintain an occupancy map (grid-based)
-- with inputs of laser points

-- Timing
local ffi = require'ffi'
local unix = require'unix'
-- local time = require 'unix'.time

local DEG_TO_RAD = require'math'.pi / 180
local abs = require'math'.abs
local cos = require'math'.cos
local sin = require'math'.sin
local ceil = require'math'.ceil
local floor = require'math'.floor
local min = require'math'.min
local max = require'math'.max
local pow = require'math'.pow
local sqrt = require'math'.sqrt
local function sign(x)
  return (x > 0 and 1) or (x < 0 and -1) or 0
end
local tinsert = require'table'.insert

local Z_MIN = 0.0254 -- 1 inch above ground

local lib = {}

local map
local map_type = "uint8_t"
-- Map dimensions in meters
local map_scale = 0.05 -- meters per cell (5cm)
local map_w = 75 -- 75 meters
local map_h = 75 -- 75 meters
-- local map_w = 55
-- local map_h = 55
-- Map dimensions in cells
local map_scale_inv -- cells per meter
local map_ih, map_iw -- cells
-- Map center
local map_ixc, map_iyc -- cell

local function derive_parameters()
  -- Map dimensions in cells
  map_scale_inv = 1 / map_scale
  map_ih = ceil(map_w * map_scale_inv)
  map_iw = ceil(map_h * map_scale_inv)
  -- Map center
  map_ixc = floor(map_iw / 2)
  map_iyc = floor(map_ih / 2)
  -- Exports
  lib.scale = map_scale
end

-- Make map fully uncertain
local function clear_map(map)
  for i=0, (map_ih * map_iw)-1 do map[i] = 127 end
end

-- TODO: Make into a self based module for "map"
function lib.init(fname)
  if type(fname)=='string' then
    lib.load_pgm(fname)
    derive_parameters()
    return
  end
  derive_parameters()
  map = ffi.new(map_type.."[?]", map_ih * map_iw)
  clear_map(map)
  local ptr = tonumber(ffi.cast('intptr_t', ffi.cast('void *', map)))
  lib.map_info = {ptr, map_ih * map_iw,
    map_iw, map_ih, ffi.sizeof(map_type)}
end

-- Matching a pose with know offsets
-- Local offsets: x should be more than y
local offset_pts = {}
for x=-48, 48, 8 do -- Units: mm
  for y=-20, 20, 5 do -- Units: mm
    if x~=0 and y~=0 then
      -- Units: meters
      tinsert(offset_pts, {x/1e3, y/1e3})
    end
  end
end
-- TODO: Add a prior on not moving
-- This is always the first entry
tinsert(offset_pts, 1, {0, 0})

-- Local offsets: theta
local offset_th = {}
for th=-8, 8, 1 do -- Units: quarter-degrees (Hokuyo step)
  if th ~= 0 then
    -- Units: radians
    tinsert(offset_th, th * DEG_TO_RAD / 4)
  end
end
tinsert(offset_th, 1, 0)

local function ij2idx(i, j)
  return i>=0 and i<map_iw and j>=0 and j<map_ih and (map_iw * j + i)
end

local function x2i(x)
  return floor(map_scale_inv * x + map_ixc)
end

local function y2j(y)
  -- return floor(map_scale_inv * y + map_iyc)
  return floor(map_iyc - map_scale_inv * y)
end

local function pt2idx(x, y)
  return ij2idx(x2i(x), y2j(y))
end

local function tf2D(x, y, c, s, tx, ty)
  return x * c + y * s + tx, x * s - y * c + ty
end

-- Binary PGM is easiest
function lib.save_pgm(fname)
  if ffi.sizeof(map_type) ~= 1 then
    return false, "Bad map type"
  elseif type(fname)~='string' then
    return false, "Bad filename"
  end
  local f_pgm = io.open(fname, "w")
  if not f_pgm then return false end
  f_pgm:write(table.concat({
    "P5",
    map_iw, map_ih, 255,
    '# map_scale: '..map_scale,
    ''}, "\n"))
  unix.fwrite(f_pgm, map, map_ih * map_iw)
  f_pgm:close()
end

function lib.load_pgm(fname)
  if ffi.sizeof(map_type) ~= 1 then
    return false, "Bad map type"
  elseif type(fname)~='string' then
    return false, "Bad filename"
  end
  local f_pgm = io.open(fname)
  if not f_pgm then return false end
  local magic = f_pgm:read"*line"
  if magic ~="P5" then return false, "Bad magic" end
  local w = tonumber(f_pgm:read"*line")
  local h = tonumber(f_pgm:read"*line")
  if not w or not h then return false, "Bad dims" end
  local max = tonumber(f_pgm:read"*line")
  if max~=255 then return false, "Bad max" end
  local scale = f_pgm:read"*line":match"# map_scale: (%S+)"
  scale = tonumber(scale)
  if not scale then return false, "No scale" end
  -- Set
  map_iw = w
  map_ih = h
  map_scale = scale
  local n_cells = map_ih * map_iw
  map = ffi.new(map_type.."[?]", n_cells)
  unix.fread(f_pgm, map, n_cells)
  f_pgm:close()
  return map
end

local function norm(pa, pb)
  local d = 0
  for i, a in ipairs(pa) do
    d = d + pow(a - pb[i], 2)
  end
  return sqrt(d)
end

-- Indicies of points that are within the voxel size
-- For now, we assume some ordering, radially
local function thindex(xs, ys, zs)
  local inds = {}
  local DIST_THRESH = 1.5 * map_scale
  -- just a 2D map, so consider only the 2D points
  -- NOTE: In the future, we can change a bit
  local p_prev = {math.huge, math.huge}
  for i=1,#xs do
    local p = {xs[i], ys[i]}
    local dp_prev = norm(p_prev, p)
    -- Constrain to the map voxel size
    if dp_prev >= DIST_THRESH then
      p_prev = p
      tinsert(inds, i)
    end
  end
  return inds
end
lib.thindex = thindex

-- Take laser points in robot frame
-- Outputs a pose offset in the robot frame
-- Samples known offsets around pose
function lib.match_pose(pose_map, pts_rbt, valid, thinds)
  local pose_x, pose_y, pose_th = unpack(pose_map)
  local lxs_rbt, lys_rbt, lzs_rbt = unpack(pts_rbt)
  local vmax_hits = -math.huge
  local imax_th = -1
  local imax_pt = -1
  -- Search across angles th(eta)
  for i_th, th in ipairs(offset_th) do
    local hits_off_pts = {}
    hits_off_pts[1] = i_th==1 and 2550 or 0
    for i=2, #offset_pts do
      hits_off_pts[i] = i_th==1 and 1275 or 0
    end
    --
    local c, s = cos(pose_th + th), sin(pose_th + th)
    -- Search translation offsets
    for i_off, offset in ipairs(offset_pts) do
      local dx, dy = unpack(offset)
      -- Iterate over the laser points
      -- TODO: Support no thinds as being all the points
      -- for i_l=1,#lxs_map do
      for _, i_l in ipairs(thinds) do
        -- Transform the lidar point from robot frame to map frame
        local px, py = tf2D(
          lxs_rbt[i_l] + dx, lys_rbt[i_l] + dy,
          c, s,
          pose_x, pose_y)
        local imap = pt2idx(px, py)
        -- Not all transformed points lie on the map
        if imap and (lzs_rbt[i_l] > Z_MIN) then
          local v = map[imap]
          hits_off_pts[i_off] = hits_off_pts[i_off] + (v > 127 and v or 0)
        end
      end
    end
    -- Add a prior to our initial guess
    local imax, vmax = -1, -math.huge
    for i, v in ipairs(hits_off_pts) do
      if v > vmax then imax = i; vmax = v end
    end
    if vmax > vmax_hits then
      imax_th = i_th
      imax_pt = imax
      vmax_hits = vmax
    end
    -- print("ith", i_th, vmax_hits)
    -- print(180*th/math.pi, vmax, unpack(offset_pts[imax]))
  end
  -- print(vmax_hits, imax_th, imax_pt)
  return vmax_hits, offset_th[imax_th], unpack(offset_pts[imax_pt])
end

-- Take laser points in robot frame
-- Outputs: best particle? updated weights for the particles?
-- Finds the best particle
function lib.match_particles(particles_map, pts_rbt)
  local lxs_rbt, lys_rbt, lzs_rbt = unpack(pts_rbt)
  --
  local hits = {}
  local imax, vmax = -1, -math.huge
  -- Iterate through each particle
  for ip, p_map in particles_map do
    local pose_x, pose_y, pose_th = unpack(p_map)
    local c, s = cos(pose_th), sin(pose_th)
    local hits_off_pts = 0
    -- Iterate through each lidar point
    -- for i_l=1,#lxs_map do
      for _, i_l in ipairs(thinds) do
      -- Transform the lidar point from robot frame to map frame
      local px, py = tf2D(
            lxs_rbt[i_l], lys_rbt[i_l],
            c, s,
            pose_x, pose_y)
      local imap = pt2idx(px, py)
      -- Not all transformed points lie on the map
      if imap and lzs_rbt[i_l] > Z_MIN then
        hits_off_pts = hits_off_pts + map[imap]
      end
    end
    if hits_off_pts > vmax then
      vmax = hits_off_pts
      imax = ip
    end
    hits[ip] = hits_off_pts
  end
  -- Return the index of the best particle
  -- and the hit rates of all particles
  return imax, hits
end

local function bresenham(i0, j0, i1, j1, delta)
  local deltax = i1 - i0
  local deltay = j1 - j0
  local deltaerr = abs(deltay / deltax)
  local dy_sign = sign(deltay)
  local err = 0
  -- Cast this point
  local j = j0
  for i = i0, i1, sign(deltax) do
    err = err + deltaerr
    while err >= 0.5 and j>=0 and j<map_ih do
      --
      local idx = ij2idx(i, j)
      if idx then map[idx] = max(0, map[idx] + delta) end
      --
      j = j + dy_sign
      err = err - 1
    end
  end
end

-- Go from pose outward
-- Input requirement: all points lie on the map, so no index checking
local function raytrace(pose_map, pts_map, delta)
  local lxs_map, lys_map = unpack(pts_map)
  local i0, j0 = x2i(pose_map[1]), y2j(pose_map[2])
  for i_lx, lx_map in ipairs(lxs_map) do
    local i1, j1 = x2i(lx_map), y2j(lys_map[i_lx])
    if i0==i1 then
      for j=min(j0,j1), max(j0,j1) do
        local idx = ij2idx(i0, j)
        if idx then map[idx] = max(0, map[idx] + delta) end
      end
    elseif j0==j1 then
      for i=min(i0,i1), max(i0,i1) do
        local idx = ij2idx(i, j0)
        if idx then map[idx] = max(0, map[idx] + delta) end
      end
    else
      bresenham(i0, j0, i1, j1, delta)
    end
  end
end

-- Update the map with LIDAR points
function lib.update(pose, pts_rbt, hits, thinds)
  local pose_x, pose_y, pose_th   = unpack(pose)
  local lxs_rbt, lys_rbt, lzs_rbt = unpack(pts_rbt)
  local c, s = cos(pose_th), sin(pose_th)

  -- Check the ratio of on-map to offmap
  local map_rate = #hits / #pts_rbt[1]
  -- TODO: If the rate is too low, then we should expand the map
  -- print(string.format("Map rate: %.2f", map_rate))

  local lxs_map, lys_map = {}, {}

  -- Increment the LIDAR hits
  -- Takes as input the laser scanner x/y points in the map frame
  -- Input requirement: all points lie on the map, so no index checking
  -- local t_increment0 = time()
  local delta = 20
  -- for i_l=1,#lxs_map do
  for _, i_l in ipairs(thinds) do
    local lx_rbt, ly_rbt = lxs_rbt[i_l], lys_rbt[i_l]
    local lx_map, ly_map = tf2D(lx_rbt, ly_rbt, c, s, pose_x, pose_y)
    -- Save for raytracing these points...
    tinsert(lxs_map, lx_map)
    tinsert(lys_map, ly_map)
    local imap = pt2idx(lx_map, ly_map)
    if imap and hits[i_l] and lzs_rbt[i_l]>Z_MIN then
      map[imap] = min(255, map[imap] + delta)
    end
  end
  -- local t_increment1 = time()
  -- print(string.format("Incremented %d in %.3f ms",
  --   #pts_map[1], (t_increment1 - t_increment0)*1e3))

  -- Raytrace
  delta = -1
  -- local t_raytrace0 = time()
  raytrace(pose, {lxs_map, lys_map}, delta)
  -- local t_raytrace1 = time()
  -- print(string.format("Raytraced %d in %.3f ms",
  --   #pts_map[1], (t_raytrace1 - t_raytrace0)*1e3))

end

return lib