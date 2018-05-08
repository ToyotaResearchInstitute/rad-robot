local lib = {}

local cos = require'math'.cos
local sin = require'math'.sin
local unpack = unpack or require'table'.unpack

local map = require'occupancy_map'
map.init()
lib.map = map
local map_scale = map.scale

local USE_LASER_MATCH = true

-- Localization state
local pose_th = 0
local pose_x = 0
local pose_y = 0

local function tf2D(x, y, c, s, tx, ty)
  return x * c + y * s + tx, x * s - y * c + ty
end

function lib.update_gyro(d_roll, d_pitch, d_yaw)
  pose_th = pose_th + (d_yaw or 0)
  return pose_x, pose_y, pose_th
end

function lib.update_odometry(dx_odom)
  local c, s = cos(pose_th), sin(pose_th)
  pose_x, pose_y = tf2D(dx_odom, 0, c, s, pose_x, pose_y)
  return pose_x, pose_y, pose_th
end

-- Input: {{x0, x1, ...}, {y0, y1, ...}}
function lib.update_laser(pts_rbt, hits)

  -- Thin the points into indices OK for the map
  local thinds = map.thindex(unpack(pts_rbt))

  -- Find the optimal offset that matches the distances
  if USE_LASER_MATCH then
    local _, dth, dx, dy = map.match_pose(
      {pose_x, pose_y, pose_th}, pts_rbt, hits, thinds)

    -- Transform the pose, based on the optimal match
    pose_th = pose_th + dth
    local c, s = cos(pose_th), sin(pose_th)
    pose_x, pose_y = tf2D(dx, dy, c, s, pose_x, pose_y)
  end

  -- Occupancy map part
  -- Update the map at this ideal pose
  map.update({pose_x, pose_y, pose_th}, pts_rbt,
    hits, thinds)
  return pose_x, pose_y, pose_th
end

function lib.get_pose()
  return pose_x, pose_y, pose_th
end

return lib
