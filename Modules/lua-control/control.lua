local lib = {}

local coyield = require'coroutine'.yield
local cos, sin = require'math'.cos, require'math'.sin
local max, min = require'math'.max, require'math'.min
local atan2 = require'math'.atan2
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack
local dist = require'vector'.distance
local tf2D = require'transform'.tf2D
local vector = require'vector'

local function gen_path(waypoints, ds)
  ds = tonumber(ds) or 0.05
  -- Two dimensional points
  local path, length = {}, 0
  for i=1, #waypoints-1 do
    local p_a = vector.new(waypoints[i])
    local p_b = vector.new(waypoints[i+1])
    local dp = p_b - p_a
    local d = vector.norm(dp)
    length = length + d
    dp = vector.unit(dp)
    tinsert(path, p_a)
    for step = ds, d-ds, ds do
      local p = p_a + step * dp
      tinsert(path, p)
    end
  end
  return path, length
end
lib.gen_path = gen_path

-- Usage: c
-- L is lookahead distance
-- threshold_close is how far away from the path before we give up
local function pure_pursuit(params)
  -- Initialization bits
  params = type(params)=='table' and params or {}
  local path = params.path
  local fn_nearby = params.fn_nearby
  if type(path)~='table' then
    return false, "Bad path"
  elseif #path==0 then
    return false, "No path points"
  elseif type(fn_nearby)~='function' then
    return false, "No nearby function"
  end
  local lookahead = tonumber(params.lookahead) or 1
  local n_path = #path
  local p_final = path[n_path]
  local id_last = tonumber(params.id_start)
  -- Give a function to be created/wrapped by coroutine
  return function(pose_rbt)
    local result
    while pose_rbt do
      -- Now, the update routine
      local x_rbt, y_rbt, th_rbt = unpack(pose_rbt)
      local d_goal = dist(pose_rbt, p_final)
      -- Find the lookahead point
      local x_ahead, y_ahead = tf2D(lookahead, 0, th_rbt, x_rbt, y_rbt)
      local p_lookahead = {x_ahead, y_ahead}
      -- Find this in the path
      local id_nearby, dist = fn_nearby(id_last, p_lookahead)
      -- Save the point for next time
      id_last = id_nearby
      -- Wait until we are close
      if not id_nearby then
        id_last = nil
        result = {err= dist}
      elseif id_nearby >= n_path and d_goal <= lookahead then
        result = {done = true}
      else
        -- Find delta between robot and lookahead path point
        local p_nearby = path[id_nearby]
        local x_ref, y_ref = unpack(p_nearby)
        -- Relative angle towards the lookahead reference point
        local alpha = atan2(y_ref - y_rbt, x_ref - x_rbt)
        alpha = alpha - th_rbt
        -- kappa is curvature (inverse of the radius of curvature)
        local kappa = 2 * sin(alpha) / lookahead
        -- Give result and take the next pose
        result = {kappa = kappa,
                  id_path = id_nearby,
                  pose_rbt = pose_rbt,
                  p_nearby = p_nearby,
                  p_lookahead = p_lookahead,
                  alpha = alpha,
                  lookahead = lookahead,
                  d_goal = d_goal,
                 }
      end
      pose_rbt = coyield(result)
    end
    return false, "No pose"
  end

end
lib.pure_pursuit = pure_pursuit

return lib
