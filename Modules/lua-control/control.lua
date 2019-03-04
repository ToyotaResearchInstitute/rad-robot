local lib = {}

local atan2 = require'math'.atan2 or require'math'.atan
local fabs = require'math'.abs
local ceil = require'math'.ceil
local cos = require'math'.cos
local sin = require'math'.sin
local sqrt = require'math'.sqrt
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack
--
local mod_angle = require'transform'.mod_angle
local tf2D = require'transform'.tf2D
--
local has_kdtree, kdtree = pcall(require, 'kdtree')

local function generate_kdtree(path)
  -- Add the kd-tree for quick access
  local tree = path.tree
  if type(tree) ~= 'userdata' and has_kdtree then
    -- 2D points
    local k_dim = 2
    tree = assert(kdtree.create(k_dim))
  end
  if not kdtree then
    return false, "No kdtree available"
  end
  for i, pt in ipairs(path) do tree:insert(pt, i) end
  if tree:size() ~= #path then
    return false, "No points added to the kd-tree"
  end
  -- Add to the
  path.tree = tree
  return tree
end
lib.generate_kdtree = generate_kdtree

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
  local steps_lookahead = ceil(lookahead / path.ds)
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
