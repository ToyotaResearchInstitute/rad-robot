local lib = {}

local atan2 = require'math'.atan2 or require'math'.atan
local fabs = require'math'.abs
local ceil = require'math'.ceil
local cos = require'math'.cos
local sin = require'math'.sin
local sqrt = require'math'.sqrt
local tan = require'math'.tan
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack
--
local mod_angle = require'transform'.mod_angle
local tf2D = require'transform'.tf2D
local tf2D_inv = require'transform'.tf2D_inv
--
local has_kdtree, kdtree = pcall(require, 'kdtree')

local function generate_control_points(waypoints)
  -- Turning radius of the rc car
  -- A turning radius of 0 gives line segments
  local turning_radius = tonumber(waypoints.turning_radius) or 0
  local closed = waypoints.closed and true or false
  local n_wp = #waypoints
  if n_wp < 2 then
    return false, "Not enough points"
  end
  -- Add the angles of approach (heading)
  local waypoints1 = {closed=closed, turning_radius=turning_radius}
  for i=1,n_wp do
    local p_a = waypoints[i]
    local heading
    if i==n_wp and not closed then
      -- Take the heading of the previous point
      local pt_previous = waypoints1[#waypoints1]
      heading = pt_previous[3]
    else
      local p_b = waypoints[i + 1] or waypoints[1]
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

  -- Find the knot points
  local knots = {closed=closed, turning_radius=turning_radius}
  if not closed then
    -- Ensure that the first point is first
    -- NOTE: For loops, it should not matter
    tinsert(knots, {da=0, unpack(waypoints1[1])})
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
      tinsert(knots, kn_1)
    else
      -- Find the offset distance before beginning turn
      local offset = fabs(turning_radius / tan((math.pi - da_approach) / 2))
      -- Add the knot point just before beginning the turn
      local c_a, s_a = cos(p_a[3]), sin(p_a[3])
      local dx_1, dy_1 = offset * c_a, offset * s_a
      local kn_1 = {p_b[1] - dx_1, p_b[2] - dy_1, p_a[3], da=da_approach}
      tinsert(knots, kn_1)
      -- Add the knot point just after completing the turn
      local c_b, s_b = cos(p_b[3]), sin(p_b[3])
      local dx_2, dy_2 = offset * c_b, offset * s_b
      local kn_2 = {p_b[1] + dx_2, p_b[2] + dy_2, p_b[3], da=0}
      tinsert(knots, kn_2)
    end
  end
  return knots, waypoints1
end
lib.generate_control_points = generate_control_points

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
  -- Clockwise (-1) or counterclockwise (1)
  local dir = a2 > a1 and 1 or -1
  -- Offset for drawing path tangent to arc
  local tangent_offset = dir * math.pi/2
  -- Cheat and set the arc length.
  -- NOTE: May be better to compute the line segments, though...
  local arc_length = rc * fabs(a2 - a1)
  local n_segments = ceil(arc_length / ds)
  local ds1 = arc_length / n_segments
  local dth1 = dir * ds1 / rc
  local th = a1
  -- Start from 0 for inclusive
  for _=0,n_segments do
    local c, s = cos(th), sin(th)
    local dx, dy = c * rc, s * rc
    local px, py = xc + dx, yc + dy
    tinsert(path, {px, py, th + tangent_offset})
    th = th + dth1
  end
  return path, arc_length
end
lib.path_arc = path_arc

local function generate_path(knots, params)
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
  local n_knots = #knots
  -- Enumerate and draw the points from the knots (control points)
  for i=1,n_knots do
    local kn_1 = knots[i]
    local kn_2 = knots[i + 1] or knots[1]
    if not closed and i==n_knots then
      break
    end
    -- Check the knot point properties
    local da_approach = kn_1.da or 0 -- or 0 to support non-knot point paths
    local px_approach, py_approach, angle_approach = unpack(kn_1, 1, 3)
    local px_exit, py_exit = unpack(kn_2, 1, 2)
    local dx_knots, dy_knots = px_exit - px_approach, py_exit - py_approach
    local d_knots = sqrt(dx_knots * dx_knots + dy_knots * dy_knots)
    -- Check what to add to the path (and raster)
    if da_approach == 0 then
      -- Add a line to the path
      local _, d_pts = path_line(kn_1, kn_2, ds, path)
      -- Pop the last element when joining paths
      -- Because kn_1 and kn_2 are inclusive
      -- Closed/Open interval concatenation, except last:
      -- [a1, b1), [a2, b2), [a3, b3), ... [a_n, b_n]
      -- TODO: Ensure thatn "arc" behaves this way, too
      if i < n_knots - 1 or closed then table.remove(path) end
      -- Increment the length
      length = length + d_pts
      -- Draw the line
      if grid_raster then grid_raster:bresenham(kn_1, kn_2) end
    elseif d_knots > ds then
      -- Draw an arc to connect the knots if curved and further than ds
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
      if i < n_knots - 1 or closed then table.remove(path) end
      -- Draw the points
      if grid_raster then grid_raster:arc(center_of_arc, turning_radius, th1, th2) end
    end
  end

  -- Add the kd-tree for quick access
  local tree = path.tree
  if type(tree) ~= 'userdata' and has_kdtree then
    -- 2D points
    local k_dim = 2
    tree = kdtree.create(k_dim)
  end
  if tree then
    for i, pt in ipairs(path) do tree:insert(pt, i) end
    if tree:size() ~= #path then
      return false, "No points added to the kd-tree"
    end
    path.tree = tree
  end

  return path, length
end
lib.generate_path = generate_path

local function sort_candidates(a, b)
  return a.dist < b.dist
end

-- Given: Point
-- Returns: id_nearby, is_within_threshold
local function get_nearest(pt, path, threshold_close, id_last)
  -- NOTE: nearby must be sorted by increasing distance
  -- If not id_last, then simply go to the nearest point?
  local nearby, err = path.tree:nearest(pt, threshold_close)
  if id_last and not nearby then
    return false, err
  elseif not id_last then
    -- Search for the closest points...
    nearby, err = path.tree:nearest(pt)
    if not nearby then return false, err end
    return nearby[1].user, false
  end
  -- Examine in sorted order, by distance
  for _, nby in ipairs(nearby) do
    -- TODO: Enforce ordering: (nby.user >= id_last)
    if not id_last then
      return nby.user, true
    end
  end
  return false, "No unvisited points"
end

-- Find in a particular lane
local function find_in_path(p_vehicle, path, closeness)
  closeness = tonumber(closeness) or 1
  -- Grab the pose angle
  local p_x, p_y, p_a = unpack(p_vehicle)
  -- Generate the candidates for this path
  local nearby, err = path.tree:nearest(p_vehicle, closeness)
  if not nearby then
    return false, err
  end
  -- Since distance sorted, find the first with a reasonable alignment
  for _, nby in ipairs(nearby) do
    local id_in_path = nby.user
    local path_x, path_y, path_a = unpack(path[id_in_path])
    local dx, dy = p_x - path_x, p_y - path_y
    -- Only if the path _has_ an angle at that point
    local da = path_a and mod_angle(p_a - path_a)
    if fabs(da) < math.rad(45) then
      return {
        id_in_path=nby.user,
        dist=sqrt(dx*dx + dy*dy),
        da=da
      }
    end
  end
  return false, "No well-oriented candidates"
end
lib.find_in_path = find_in_path

-- Given a pose and a table of paths
local function find_in_paths(p_vehicle, paths, closeness, skip_angle)
  closeness = tonumber(closeness) or 1
  -- Grab the pose angle
  local p_x, p_y, p_a = unpack(p_vehicle)
  -- Generate the best candidate per path
  local candidates = {}
  for k, path in pairs(paths) do
    local nearby, err = path.tree:nearest(p_vehicle, closeness)
    if nearby then
      for _, nby in ipairs(nearby) do
        local id_in_path = nby.user
        local path_x, path_y, path_a = unpack(path[id_in_path])
        local dx, dy = p_x - path_x, p_y - path_y
        -- Only if the path _has_ an angle at that point
        local da = path_a and mod_angle(p_a - path_a)
        tinsert(candidates, {
          path_name=k,
          id_in_path=nby.user,
          dist=sqrt(dx*dx + dy*dy),
          da=da})
      end
    end
  end
  if #candidates==0 then
    return false, "No candidates"
  end
  -- Sort all candidates by distance - across paths
  table.sort(candidates, sort_candidates)
  -- Now check on the angle
  if skip_angle then
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

-- Usage:
-- lookahead distance
-- threshold_close is how far away from the path before we give up
local function pure_pursuit(params)
  -- Initialization bits
  if type(params) ~= 'table' then return false, "No parameters" end
  local path = params.path
  if type(path)~='table' then
    return false, "Bad path"
  elseif #path==0 then
    return false, "No path points"
  end
  local threshold_close = tonumber(params.threshold_close) or 0.25
  local lookahead = tonumber(params.lookahead) or 1
  local steps_lookahead = math.ceil(lookahead / path.ds)
  local id_lookahead_last = false
  local fn_nearby = params.fn_nearby
  if type(fn_nearby) ~= "function" then fn_nearby = get_nearest end
  -- Give a function to be created/wrapped by coroutine
  -- Input: pose_rbt
  -- State: result_prev
  local function controller(pose_rbt, result_prev)
    -- Find ourselves on the path
    local id_last = result_prev and result_prev.id_path
    local id_path, is_nearby = fn_nearby(pose_rbt, path, threshold_close, id_last)
    -- If we can't find anywhere on the path to go... Bad news :(
    if not id_path then
      return false, is_nearby
    end
    local p_path = path[id_path]
    local d_path = false
    local x_rbt, y_rbt, th_rbt = unpack(pose_rbt)
    if p_path then
      local x_path, y_path = unpack(p_path, 1, 2)
      local dx, dy = x_rbt - x_path, y_rbt - y_path
      -- Distance to the path
      d_path = sqrt(dx * dx + dy * dy)
    end
    -- Prepare the result
    local result = {
      id_path = id_path,
      p_path = p_path,
      d_path = id_path and d_path,
      id_last = id_last,
      id_lookahead_last = id_lookahead_last,
      lookahead = lookahead,
    }
    if id_path==#path and not path.closed then
      result.done = true
      return result
    end
    -- Check if we are nearby the path _and_ we are well oriented
    local id_lookahead, p_lookahead
    if is_nearby then
      -- Prepare the lookahead point
      local x_ahead, y_ahead = tf2D(lookahead, 0, th_rbt, x_rbt, y_rbt)
      p_lookahead = {x_ahead, y_ahead}
      -- Find the nearest path point to the lookahead point
      -- TODO: Don't skip too far in a single timestep?
      id_lookahead = fn_nearby(p_lookahead, path, threshold_close, id_lookahead_last)
    end
    -- Default to one point ahead
    if not id_lookahead then
      -- Keep in a loop using the modulo
      -- print("steps_lookahead", steps_lookahead)
      id_lookahead = id_path + steps_lookahead
      if id_lookahead > #path then
        if path.closed then
          -- Assuming that steps_lookahead < #path
          id_lookahead = id_lookahead - #path
        else
          id_lookahead = #path
        end
      end
      -- print("id_path", id_path)
      -- print("id_lookahead", id_lookahead)
      p_lookahead = path[id_lookahead]
      -- print("p_path", unpack(p_path))
      -- print("p_lookahead", unpack(p_lookahead))
      -- error("OOPS")
    end
    result.p_lookahead = p_lookahead
    -- Find delta between robot and lookahead path point
    local p_lookahead_path = assert(path[id_lookahead], "No lookahead point on the path")
    -- Ensure that we set this variable for debugging
    if not id_lookahead then
      result.p_lookahead = p_lookahead_path
    end
    local x_ref, y_ref = unpack(p_lookahead_path)
    -- Relative angle towards the lookahead reference point
    local alpha = atan2(y_ref - y_rbt, x_ref - x_rbt)
    alpha = alpha - th_rbt
    -- kappa is curvature (inverse of the radius of curvature)
    local two_sa = 2 * sin(alpha)
    local kappa = two_sa / lookahead
    local radius_of_curvature = lookahead / two_sa
    -- Add results
    result.kappa = kappa
    result.radius_of_curvature = radius_of_curvature
    result.id_lookahead = id_lookahead
    result.p_lookahead_path = p_lookahead_path
    -- result.d_lookahead = d_lookahead
    result.alpha = alpha
    result.err = nil
    -- Save the nearby point for next time
    id_lookahead_last = id_lookahead
    return result
  end
  return controller
end
lib.pure_pursuit = pure_pursuit

-- https://en.wikipedia.org/wiki/PID_controller#PID_controller_theory
-- TODO: Orientation...
local function pid(params)
  -- Default params have P control with gain of 1
  if type(params)~='table' then params = {} end
  -- r is the desired value (setpoint)
  -- Default drives to zero
  local r = tonumber(params.r) or 0
  -- Default drives to zero
  local r_dot = tonumber(params.r_dot) or 0
  -- Default to P control
  local Kp = tonumber(params.Kp) or 1
  local Ki = tonumber(params.Ki) or 0
  local Kd = tonumber(params.Kd) or 0
  -- Closure for accumulation of error
  local e_last = 0
  local e_total = 0
  -- Function takes y as the measured value
  -- Optionally, take in the measured derivative
  local function controller(y, y_dot)
    -- Error is difference between setpoint and present value
    local e_p = r - y
    e_total = e_total + e_p
    -- Optionally use the desired change in setpoint derivative
    local e_d = y_dot and (r_dot - y_dot) or (e_last - e_p)
    e_last = e_p
    --
    local u_p = Kp * e_p
    local u_i = Ki * e_total
    local u_d = Kd * e_d
    -- Return the control and the error state
    local u = u_p + u_i + u_d
    return u, e_p, e_total
  end
  return controller
end
lib.pid = pid

return lib
