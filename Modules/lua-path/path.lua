local lib = {}
--
local atan2 = require'math'.atan2 or require'math'.atan
local fabs = require'math'.abs
local ceil = require'math'.ceil
local cos = require'math'.cos
local sin = require'math'.sin
local sqrt = require'math'.sqrt
local tan = require'math'.tan
--
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack
--
-- local has_dubins, dubins = pcall(require, 'dubins')
local has_grid, grid = pcall(require, 'grid')
local has_kdtree, kdtree = pcall(require, 'kdtree')
--
local mod_angle = require'transform'.mod_angle
local tf2D_inv = require'transform'.tf2D_inv

local function tostring_path(p)
  return string.format("Path | %d points", #p.points)
end

local mt = {
  __tostring = tostring_path
}

local function generate_kdtree(self)
  -- Add the kd-tree for quick access
  if not has_kdtree then return false, "No kdtree available: "..tostring(kdtree) end
  -- Check if the tree already exists with the correct number of points
  if type(self.tree) == 'userdata' and self.tree:size() == #self.points then
    return self
  end
  -- 2D points
  local k_dim = 2
  local tree = assert(kdtree.create(k_dim))
  for i, pt in ipairs(self.points) do tree:insert(pt, i) end
  if tree:size() ~= #self.points then
    return false, "Not enough points added to the kd-tree"
  end
  self.tree = tree
  return self
end
lib.generate_kdtree = generate_kdtree

-- Given: Point
local function get_nearest(self, pt, threshold_close, fn_reduce)
  -- NOTE: nearby must be sorted by increasing distance
  local nearby, err = self.tree:nearest(pt, threshold_close)
  if not nearby then return false, err end
  if type(fn_reduce) ~= 'function' then return nearby[1].user end
  -- Examine in sorted order, by distance
  local done, running
  for _, nby in ipairs(nearby) do
    running, done = fn_reduce(nby.user, running)
    if done then return running end
  end
  return running
end

-- Use a kdtree for finding in a path
local function find_in_path(self, p_vehicle, options)
  if type(options) ~= 'table' then options = {} end
  local closeness = tonumber(options.closeness) or 1
  -- Grab the pose angle
  local p_x, p_y, p_a = unpack(p_vehicle)
  -- Generate the candidates for this path
  local nearby, err = self.tree:nearest(p_vehicle, closeness)
  if not nearby then return false, err end
  -- Since distance sorted, find the first with a reasonable alignment
  for _, nby in ipairs(nearby) do
    local id_in_path = nby.user
    local path_x, path_y, path_a = unpack(self.points[id_in_path])
    local dx, dy = p_x - path_x, p_y - path_y
    -- Only if the path _has_ an angle at that point
    local da = path_a and mod_angle(p_a - path_a) or 0
    local ORIENTATION_THRESHOLD = math.rad(45)
    if fabs(da) < ORIENTATION_THRESHOLD then
      return {
        idx_path = nby.user,
        idx_lane = false,
        --
        dist = sqrt(dx*dx + dy*dy),
        dist_normal = 0/0,
        dist_tangent = 0/0,
        da = da
      }
    end
  end
  return false, "No well-oriented candidates"
end
lib.find_in_path = find_in_path

local function get_id_ahead(self, id_path, lookahead)
  -- Default to one point ahead
  -- if not id_lookahead then
  local steps_lookahead = ceil(lookahead / self.ds)
  -- print("steps_lookahead", steps_lookahead)
  -- Keep in a loop using the modulo
  local id_path_lookahead = id_path + steps_lookahead
  local n_points = #self.points
  if id_path_lookahead > n_points then
    if self.closed then
      id_path_lookahead = id_path_lookahead % n_points
    else
      id_path_lookahead = n_points
    end
  end
  return id_path_lookahead
end

local function wrap(obj)
  -- Add methods to the table
  obj.find = find_in_path
  obj.generate_kdtree = generate_kdtree
  obj.nearby = get_nearest
  obj.get_id_ahead = get_id_ahead
  return setmetatable(obj, mt)
end
lib.wrap = wrap

local function sort_candidates(a, b)
  return a.dist < b.dist
end
-- Given a pose and a table of paths
local function find_in_paths(p_vehicle, paths, options)
  -- Generate the best candidate per path
  local candidates = {}
  for name_path, my_path in pairs(paths) do
    local candidate, err = find_in_path(p_vehicle, my_path, options)
    if candidate then
      candidate.path_name = name_path
      table.insert(candidates, candidate)
    end
  end
  if #candidates==0 then
    return false, "No candidates"
  end
  -- Sort all candidates by distance - across paths
  table.sort(candidates, sort_candidates)
  -- Now check on the angle
  if options.skip_angle then
    return candidates[1]
  end
  for _, candidate in ipairs(candidates) do
    local da = assert(candidate.da, "No path angle information")
    if fabs(da) < math.rad(45) then
      return candidate
    end
  end
  return false, "No well-oriented candidates"
end
lib.find_in_paths = find_in_paths

local function generate_waypoints(knotpoints)
  -- Returns a set of se(2) points from a set of knotpoints

  -- Turning radius of the rc car
  -- A turning radius of 0 gives line segments
  local turning_radius = tonumber(knotpoints.turning_radius) or 0
  local closed = knotpoints.closed and true or false
  local n_wp = #knotpoints
  if n_wp < 2 then
    return false, "Not enough points"
  end
  -- Add the angles of approach (heading)
  local waypoints1 = {closed=closed, turning_radius=turning_radius}
  for i=1,n_wp do
    local p_a = knotpoints[i]
    local heading
    if i==n_wp and not closed then
      -- Take the heading of the previous point
      local pt_previous = waypoints1[#waypoints1]
      heading = pt_previous[3]
    else
      local p_b = knotpoints[i + 1] or knotpoints[1]
      local dx, dy = p_b[1] - p_a[1], p_b[2] - p_a[2]
      heading = atan2(dy, dx)
    end
    -- Add points as {x, y, heading} in the global frame
    tinsert(waypoints1, {p_a[1], p_a[2], heading})
  end

  -- Find the differences in angles of approach
  local approach_diffs = {}
  for i=1,n_wp do
    -- If open and the last element, then approach difference is 0
    local da_approach = 0
    -- Else, calculate with regards to the next point
    if i < n_wp or closed then
      local p_a = waypoints1[i]
      local p_b = waypoints1[i + 1] or waypoints1[1]
      da_approach = mod_angle(p_b[3] - p_a[3])
    end
    tinsert(approach_diffs, da_approach)
  end

  -- Find the waypoints
  local waypoints = {closed=closed, turning_radius=turning_radius}
  if not closed then
    -- Ensure that the first point is first
    -- NOTE: For loops, it should not matter
    tinsert(waypoints, {da=0, unpack(waypoints1[1])})
  end
  for i=1,n_wp do
    -- We have added the first point, already
    if not closed and i==n_wp then break end
    -- Find the difference in angle of approach
    local da_approach = approach_diffs[i]
    -- Grab the points
    local p_a = waypoints1[i]
    local p_b = waypoints1[i + 1] or waypoints1[1]
    -- No angle change between our point and the next
    if da_approach==0 then
      local kn_1 = {p_b[1], p_b[2], p_a[3], da=0}
      tinsert(waypoints, kn_1)
    else
      -- Find the offset distance before beginning turn
      local offset = fabs(turning_radius / tan((math.pi - da_approach) / 2))
      -- Add the knot point just before beginning the turn
      local c_a, s_a = cos(p_a[3]), sin(p_a[3])
      local dx_1, dy_1 = offset * c_a, offset * s_a
      local kn_1 = {p_b[1] - dx_1, p_b[2] - dy_1, p_a[3], da=da_approach}
      tinsert(waypoints, kn_1)
      -- Add the knot point just after completing the turn
      local c_b, s_b = cos(p_b[3]), sin(p_b[3])
      local dx_2, dy_2 = offset * c_b, offset * s_b
      local kn_2 = {p_b[1] + dx_2, p_b[2] + dy_2, p_b[3], da=0}
      tinsert(waypoints, kn_2)
    end
  end
  return waypoints, waypoints1
end
lib.generate_waypoints = generate_waypoints

-- Return the path, path length and step size
local function path_line(p_a, p_b, ds, points)
  -- p_a: start point (inclusive)
  -- p_b: stop point (inclusive)
  if type(points) ~= 'table' then points = {} end
  local x1, y1, a1 = unpack(p_a)
  local x2, y2 = unpack(p_b)
  local dx, dy = x2 - x1, y2 - y1
  local dth = atan2(dy, dx)
  local d_pts = sqrt(dx * dx + dy * dy)
  local n_segments = ceil(d_pts / ds)
  -- Find the increments
  local ds1 = d_pts / n_segments
  local dx1, dy1 = dx / n_segments, dy / n_segments
  -- Start our cursor
  local p_cur = {x1, y1, a1}
  -- Include the first point in our path
  tinsert(points, p_cur)
  for _=1,n_segments do
    -- World frame x, y and the angle
    p_cur = {p_cur[1] + dx1, p_cur[2] + dy1, dth}
    tinsert(points, p_cur)
  end
  return wrap{
    points = points,
    length = d_pts,
    ds = ds1,
  }
end
lib.path_line = path_line

local function path_arc(pc, rc, a1, a2, ds, points)
  -- pc: Center point
  -- rc: Radius from center
  -- a1: start angle (inclusive)
  -- a2: stop angle (inclusive)
  -- ds: path increment
  if type(points) ~= 'table' then points = {} end
  local xc, yc = unpack(pc, 1, 2)
  -- Cheat and set the arc length.
  -- NOTE: May be better to compute the line segments, though...
  local arc_length = rc * fabs(a2 - a1)
  local n_segments = ceil(arc_length / ds)
  local ds1 = arc_length / n_segments
  -- Clockwise (-1) or counterclockwise (1)
  local dir = a2 > a1 and 1 or -1
  local ang_resolution = dir * ds1 / rc
  -- Offset for drawing path tangent to arc
  local tangent_offset = dir * math.pi/2
  -- Start from 0 for inclusive
  local th = a1
  -- for th = a1, a2, ang_resolution do
  for _=0,n_segments do
    local c, s = cos(th), sin(th)
    local dx, dy = c * rc, s * rc
    local px, py = xc + dx, yc + dy
    -- TODO: mod_angle... could do later, though
    local se2 = {px, py, th + tangent_offset}
    tinsert(points, se2)
    th = th + ang_resolution
  end
  -- SVG
  -- <path d="M80 80
  --          A 45 45, 0, 0, 0, 125 125
  --          L 125 80 Z" fill="green"/>
  -- local arc = table.concat({
  --   -- Move to the center of the arc
  --   string.format("M %f %f", unpack(rc)),
  --   string.format("A %f %f, 0, 0, 0, %f %f", px1, py1, px2, py2),
  --   string.format("M %f %f", px2, py2),
  --   }, ' ')

  return wrap{
    points = points,
    length = arc_length,
    ds = ds1,
  }
end
lib.path_arc = path_arc

-- Given a list of waypoints, generate a path
-- TODO: Add dubins, in case a check fails...
local function path_from_waypoints(waypoints, params)
  local points = params.points
  if type(points) ~= 'table' then points = {} end
  local grid_raster = params.grid_raster
  if type(grid_raster) ~= 'table' then grid_raster = false end
  local closed = params.closed and true or false
  -- ds: discrete distance between each waypoint
  local ds = params.ds
  -- Save the path length from this call
  local length = 0
  local n_waypoints = #waypoints
  -- Enumerate and draw the points from the waypoints
  for i=1,n_waypoints do
    local wp_1 = waypoints[i]
    local wp_2 = waypoints[i + 1] or waypoints[1]
    if not closed and i==n_waypoints then
      break
    end
    -- Check the waypoint properties
    local da_approach = wp_1.da or 0 -- or 0 to support redundant waypoints
    local px_approach, py_approach, angle_approach = unpack(wp_1, 1, 3)
    local px_exit, py_exit = unpack(wp_2, 1, 2)
    local dx_waypoints, dy_waypoints = px_exit - px_approach, py_exit - py_approach
    local d_waypoints = sqrt(dx_waypoints * dx_waypoints + dy_waypoints * dy_waypoints)
    -- Check what to add to the path (and raster)
    if da_approach == 0 then
      -- Add a line to the path
      local info_line = path_line(wp_1, wp_2, ds, points)
      -- Pop the last element when joining paths
      -- Because wp_1 and wp_2 are inclusive
      -- Closed/Open interval concatenation, except last:
      -- [a1, b1), [a2, b2), [a3, b3), ... [a_n, b_n]
      -- TODO: Ensure that "arc" behaves this way, too
      if i < n_waypoints - 1 or closed then table.remove(points) end
      -- Increment the length
      length = length + info_line.length
      -- Draw the line
      if grid_raster then grid_raster:bresenham(wp_1, wp_2) end
    elseif d_waypoints > ds then
      -- Draw an arc to connect the waypoints if curved and further than ds
      local dx_rel = tf2D_inv(px_exit, py_exit,
                              angle_approach, px_approach, py_approach)
      local turning_radius = fabs(dx_rel / sin(da_approach))
      --
      local dir = da_approach > 0 and 1 or -1
      local c_1, s_1 = cos(angle_approach), sin(angle_approach)
      local dx = turning_radius * c_1
      local dy = turning_radius * s_1
      local center_of_arc = {px_approach - dir * dy, py_approach + dir * dx}
      -- Where to begin drawing the arc
      -- (offset by 90 for beginning tanget to us)
      local th1 = angle_approach - dir * math.pi/2
      local th2 = th1 + da_approach
      -- Add the arc to the path
      path_arc(center_of_arc, turning_radius, th1, th2, ds, points)
      -- Pop the last element when joining paths
      if i < n_waypoints - 1 or closed then table.remove(points) end
      -- Draw the points
      if grid_raster then grid_raster:arc(center_of_arc, turning_radius, th1, th2) end
    end
  end

  return wrap{
    points = points,
    length = length,
    ds = ds,
  }
end
lib.path_from_waypoints = path_from_waypoints

local function draw_svg(path)
  return false, "Not implemented"
end
lib.draw_svg = draw_svg

local function draw_grid(self)
  -- TODO: Add a grid option
  local xmin, xmax = math.huge, -math.huge
  local ymin, ymax = math.huge, -math.huge
  for _, p in ipairs(self.points) do
    local px, py = unpack(p, 1, 2)
    xmin = math.min(xmin, px)
    xmax = math.max(xmax, px)
    ymin = math.min(ymin, py)
    ymax = math.max(ymax, py)
  end
  -- Temp grid
  local scale = 0.01 -- 1cm resolution
  local my_grid, my_err = grid.new{
    scale = scale,
    xmin = xmin - 2 * scale, xmax = xmax + 2 * scale,
    ymin = ymin - 2 * scale, ymax = ymax + 2 * scale
  }
  if not my_grid then return false, my_err end
  -- Raster the path
  my_grid:path(self.points)
  return my_grid
end
lib.draw_grid = has_grid and draw_grid

return lib