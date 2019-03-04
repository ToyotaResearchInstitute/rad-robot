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
local has_dubins, dubins = pcall(require, 'dubins')
local has_grid, grid = pcall(require, 'grid')
--
local mod_angle = require'transform'.mod_angle
local tf2D_inv = require'transform'.tf2D_inv

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
local function path_line(p_a, p_b, ds, path)
  -- p_a: start point (inclusive)
  -- p_b: stop point (inclusive)
  if type(path) ~= 'table' then path = {} end
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
  tinsert(path, p_cur)
  for _=1,n_segments do
    -- World frame x, y and the angle
    p_cur = {p_cur[1] + dx1, p_cur[2] + dy1, dth}
    tinsert(path, p_cur)
  end
  return path, d_pts, ds1
end
lib.path_line = path_line

local function path_arc(pc, rc, a1, a2, ds, path)
  -- pc: Center point
  -- rc: Radius from center
  -- a1: start angle (inclusive)
  -- a2: stop angle (inclusive)
  -- ds: path increment
  if type(path) ~= 'table' then path = {} end
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
    tinsert(path, se2)
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

  return path, arc_length
end
lib.path_arc = path_arc

-- Given a list of waypoints, generate a path
-- TODO: Add dubins, in case a check fails...
local function path_from_waypoints(waypoints, params)
  local path = params.path
  if type(path) ~= 'table' then path = {} end
  local grid_raster = params.grid_raster
  if type(grid_raster) ~= 'table' then grid_raster = false end
  local closed = params.closed and true or false
  path.closed = closed
  -- ds: discrete distance between each waypoint
  local ds = params.ds
  path.ds = ds
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
      local _, d_pts = path_line(wp_1, wp_2, ds, path)
      -- Pop the last element when joining paths
      -- Because wp_1 and wp_2 are inclusive
      -- Closed/Open interval concatenation, except last:
      -- [a1, b1), [a2, b2), [a3, b3), ... [a_n, b_n]
      -- TODO: Ensure that "arc" behaves this way, too
      if i < n_waypoints - 1 or closed then table.remove(path) end
      -- Increment the length
      length = length + d_pts
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
      path_arc(center_of_arc, turning_radius, th1, th2, ds, path)
      -- Pop the last element when joining paths
      if i < n_waypoints - 1 or closed then table.remove(path) end
      -- Draw the points
      if grid_raster then grid_raster:arc(center_of_arc, turning_radius, th1, th2) end
    end
  end

  return path, length
end
lib.path_from_waypoints = path_from_waypoints

local function draw_svg(path)
  return false, "Not implemented"
end
lib.draw_svg = draw_svg

local function draw_grid(path)
  local xmin, xmax = math.huge, -math.huge
  local ymin, ymax = math.huge, -math.huge
  for _, p in ipairs(path) do
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
  if not my_grid then
    return false, my_err
  end
  -- Raster the path
  my_grid:path(path)
  return my_grid
end
lib.draw_grid = has_grid and draw_grid

return lib