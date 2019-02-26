-- (c) 2013 Stephen McGill
-- (c) 2018 Stephen McGill
-- Maintain an occupancy map (grid-based)
-- with inputs of laser points

local cos = require'math'.cos
local sin = require'math'.sin
local min = require'math'.min
local max = require'math'.max
local pow = require'math'.pow
local sqrt = require'math'.sqrt
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack

--
local grid = require'grid'

local M_SQRT2 = sqrt(2)
local Z_MIN = 0.0254 -- 1 inch above ground

local lib = {}

-- Pose Utilities
local function tf2D(x, y, c, s, tx, ty)
  return x * c + y * s + tx, x * s - y * c + ty
end
local function dist(pa, pb)
  local d = 0
  for i, a in ipairs(pa) do
    d = d + pow(a - pb[i], 2)
  end
  return sqrt(d)
end

-- Indicies of points that are within the voxel size
-- For now, we assume some ordering, radially
-- Should be in the robot frame...
-- Separate function for world frame filtering (e.g. z < ground)
local function thindex(self, xs, ys, zs)
  local inds = {}
  local DIST_THRESH = M_SQRT2 * self.gridmap.scale
  -- just a 2D map, so consider only the 2D points
  -- NOTE: In the future, we can change a bit
  local p_prev = {math.huge, math.huge}
  for i=1,#xs do
    local p = {xs[i], ys[i]}
    local dp_prev = dist(p_prev, p)
    -- Constrain to the map voxel size
    if dp_prev >= DIST_THRESH then
      p_prev = p
      tinsert(inds, i)
    end
  end
  return inds
end

-- Take laser points in robot frame
-- Outputs: best particle? updated weights for the particles?
-- Finds the best particle
local function match_particles(self, particles_map, pts_rbt, thinds)
  local map = self.gridmap
  local ptr = map.grid
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
    for _, i_l in ipairs(thinds) do
      if lzs_rbt[i_l] > Z_MIN then
        -- Transform the lidar point from robot frame to map frame
        local px, py = tf2D(
              lxs_rbt[i_l], lys_rbt[i_l],
              c, s,
              pose_x, pose_y)
        local imap = map:xy2idx(px, py)
        -- Not all transformed points lie on the map
        if imap and ptr[imap] > 127 then
          hits_off_pts = hits_off_pts + ptr[imap]
        end
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

-- Take laser points in robot frame
-- Outputs a pose offset in the robot frame
-- Samples known offsets around pose
local function match_pose(self, pose_map, pts_rbt, valid, thinds)
  local map = self.gridmap
  local ptr = map.grid
  local offset_pts, offset_th = self.offset_pts, self.offset_th
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
        if lzs_rbt[i_l] > Z_MIN then
          -- Transform the lidar point from robot frame to map frame
          local px, py = tf2D(
            lxs_rbt[i_l] + dx, lys_rbt[i_l] + dy,
            c, s,
            pose_x, pose_y)
          local imap = map:xy2idx(px, py)
          -- Not all transformed points lie on the map
          if imap and ptr[imap] > 127 then
            hits_off_pts[i_off] = hits_off_pts[i_off] + ptr[imap]
          end
        end -- z height
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

-- Update the map with LIDAR points
local function update_map(self, pose_map, pts_rbt, hits, thinds)
  local map = self.gridmap
  local ptr = map.grid
  local update_clear, update_hit = self.update_clear, self.update_hit
  local pose_x, pose_y, pose_th   = unpack(pose_map)
  local lxs_rbt, lys_rbt, lzs_rbt = unpack(pts_rbt)
  local c, s = cos(pose_th), sin(pose_th)

  -- Check the ratio of on-map to offmap
  -- TODO: If the rate is too low, then we should expand the map
  -- local map_rate = #hits / #pts_rbt[1]
  -- print(string.format("Map rate: %.2f", map_rate))

  -- TODO: Save the valid points for later debugging purposes
  -- local lxs_map, lys_map = {}, {}

  -- Increment the LIDAR hits
  -- Takes as input the laser scanner x/y points in the map frame
  -- Input requirement: all points lie on the map, so no index checking
  for _, i_l in ipairs(thinds) do
    local lx_rbt, ly_rbt = lxs_rbt[i_l], lys_rbt[i_l]
    local lx_map, ly_map = tf2D(lx_rbt, ly_rbt, c, s, pose_x, pose_y)
    -- Check if there is a hit (hits[i_l]) and above ground
    if hits[i_l] and lzs_rbt[i_l] > Z_MIN then
      -- Raytrace
      -- NOTE: This raytraces the hit point itself, in addition to the ray
      map:bresenham(pose_map, {lx_map, ly_map}, update_clear)
      -- Grab the index and ensure the point lies within our map bounds
      local imap = map:xy2idx(lx_map, ly_map)
      if imap then
        -- Increment the map point of the hit
        update_hit(ptr, imap)
      end
    end
    -- End the loop
  end
end

function lib.init(params)
  local gridmap = grid.new(params)
  -- Fully Uncertain
  gridmap:fill(127)
  -- Matching a pose with known offsets
  -- Local offsets: x should be more than y
  local offset_pts = {}
  -- Zero is always the first entry
  tinsert(offset_pts, {0, 0})
  --
  for x=-48, 48, 8 do -- Units: mm
    for y=-20, 20, 5 do -- Units: mm
      if x~=0 and y~=0 then
        -- Units: meters
        tinsert(offset_pts, {x/1e3, y/1e3})
      end
    end
  end
  -- Local offsets: theta
  local offset_th = {}
  -- Zero is always the first entry
  tinsert(offset_th, 0)
  for th=-8, 8, 1 do -- Units: quarter-degrees (Hokuyo step)
    if th ~= 0 then
      -- Units: radians
      tinsert(offset_th, math.rad(th * 0.25))
    end
  end

  -- Operating on a uint8_t map, so band betweeen 0 and 255
  local delta_hit = 20
  local function update_hit(ptr, idx)
    ptr[idx] = min(max(0, ptr[idx] + delta_hit), 255)
  end
  local delta_clear = -1
  local function update_clear(ptr, idx)
    ptr[idx] = min(max(0, ptr[idx] + delta_clear), 255)
  end

  return {
    gridmap = gridmap,
    --
    offset_pts = offset_pts,
    thindex = thindex,
    --
    match_particles = match_particles,
    match_pose = match_pose,
    update_map = update_map,
    --
    update_hit = update_hit,
    update_clear = update_clear
  }
end

return lib