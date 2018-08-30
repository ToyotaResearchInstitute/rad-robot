local lib = {}

local coyield = require'coroutine'.yield
local cos, sin = require'math'.cos, require'math'.sin
local max, min = require'math'.max, require'math'.min
local atan2 = require'math'.atan2
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack
local dist = require'vector'.distance
local tf2D = require'transform'.tf2D
local kdtree = require'kdtree'

-- Usage: c
-- L is lookahead distance
-- threshold_close is how far away from the path before we give up
-- TODO: Make lookahead a function - shuld use the curvature of the car, currently
local function pure_pursuit(path, lookahead, threshold_close)
  -- Initialization bits
  threshold_close = tonumber(threshold_close) or 0.5 -- meters
  -- Two dimensional points
  local tree = kdtree.create(2)
  -- Populate the tree with the points on the path
  for i, p in ipairs(path) do tree:insert(p, i) end
  local p_final = path[#path]
  local id_path_last
  -- Give a function to be created/wrapped by coroutine
  return function(pose_rbt)
    while pose_rbt do
      -- Now, the update routine
      local x_rbt, y_rbt, th_rbt = unpack(pose_rbt)
      -- Find the lookahead point
      local x_ahead, y_ahead = tf2D(lookahead, 0, th_rbt, x_rbt, y_rbt)
      local p_lookahead = {x_ahead, y_ahead}
      -- Find this in the path
      local nearby = tree:nearest(p_lookahead, threshold_close)
      if not nearby then
        --return false, string.format("Too far: (%.2f, %.2f) %.2f",
        --                            x_ahead, y_ahead, threshold_close)
        id_path_last = nil
        coyield(string.format("Too far: (%.2f, %.2f) %.2f",x_ahead, y_ahead, threshold_close))
      else
      -- Lookahead point does not go backwards
      if not id_path_last then
        id_path = nearby[1].user
      else
	-- Sort by distance
        -- table.sort(nearby, function(a, b) return a.dist_sq < b.dist_sq end)
        for _, nby in ipairs(nearby) do
          if nby.user >= id_path_last then
            id_path = nby.user
            break -- Take the first one
          end
        end
	-- Don't skip too far in a single timestep
	-- TODO: Fancier implementation
        id_path = min(id_path, id_path_last + 1)
      end
      -- Check if we are done
      local d_goal = dist(pose_rbt, p_final)
      if id_path>=#path and d_goal <= lookahead then
        return {done=true}
      end
      -- The reference point is the nearby one furthest along in the path
      -- Save the point for next time
      id_path_last = id_path
      local p_near = path[id_path]
      -- Generate the heading
      local x_ref, y_ref = unpack(p_near)
      local dx = x_ref - x_rbt
      local dy = y_ref - y_rbt
      -- Relative angle towards the lookahead reference point
      local alpha = atan2(dy, dx) - th_rbt
      -- kappa is curvature (inverse of the radius of curvature)
      local kappa = 2 * sin(alpha) / lookahead
      -- Give result and take the next pose
      pose_rbt = coyield{kappa = kappa,
                         id_path = id_path,
                         pose_rbt = pose_rbt,
                         p_near = p_near,
                         p_lookahead = p_lookahead,
                         alpha = alpha,
                         lookahead = lookahead,
                         d_goal = d_goal,
                         }

      end
    end
    return false, "No pose"
  end

end
lib.pure_pursuit = pure_pursuit

return lib
