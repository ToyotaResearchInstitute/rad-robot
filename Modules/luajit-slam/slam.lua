local lib = {}

local cos = require'math'.cos
local sin = require'math'.sin
local unpack = unpack or require'table'.unpack

local occupancy_map = require'occupancy_map'

local function tf2D(x, y, c, s, tx, ty)
  return x * c + y * s + tx, x * s - y * c + ty
end

local function update_gyro(self, d_gyro)
  self.pose_th = self.pose_th + d_gyro[3]
  return self
end

local function update_odometry(self, d_odom)
  -- d_odom: {dx, dy, dth}
  self.pose_x, self.pose_y = tf2D(d_odom[1], d_odom[2],
    cos(self.pose_th), sin(self.pose_th),
    self.pose_x, self.pose_y)
  return self
end

-- Input: {{x0, x1, ...}, {y0, y1, ...}}
local function update_laser(self, pts_rbt, hits)
  -- Thin the points into indices OK for the map
  local thinds = self.omap:thindex(unpack(pts_rbt))

  -- Find the optimal offset that matches the distances
  if self.use_laser_match then
    local _, dth, dx, dy = self.omap:match_pose(
      {self.pose_x, self.pose_y, self.pose_th}, pts_rbt, hits, thinds)

    -- Transform the pose, based on the optimal match
    self.pose_th = self.pose_th + dth
    self.pose_x, self.pose_y = tf2D(dx, dy,
      cos(self.pose_th), sin(self.pose_th),
      self.pose_x, self.pose_y)
  end

  -- Occupancy map part
  -- Update the map at this ideal pose
  self.omap:update_map({self.pose_x, self.pose_y, self.pose_th},
    pts_rbt, hits, thinds)
  return self
end

local function get_pose(self)
  return {self.pose_x, self.pose_y, self.pose_th}
end

function lib.new(parameters)
  local omap = occupancy_map.new()
  return {
    -- Occupancy Map
    omap = omap,
    -- Localization state
    pose_th = 0,
    pose_x = 0,
    pose_y = 0,
    -- Methods
    update_gyro = update_gyro,
    update_laser = update_laser,
    update_odometry = update_odometry,
    get_pose = get_pose,
    --
    use_laser_match = not parameters.odometry_only
  }
end

return lib
