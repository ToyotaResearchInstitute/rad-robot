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
local tf2D = require'transform'.tf2D

-- Pure Pursuit
local function get_inverse_curvature(pose_rbt, p_lookahead)
  local x_rbt, y_rbt, th_rbt = unpack(pose_rbt)
  local x_ref, y_ref = unpack(p_lookahead)
  -- Lookahead distance
  local dx, dy = x_ref - x_rbt, y_ref - y_rbt
  local lookahead = sqrt(dx*dx + dy*dy)
  -- Relative angle towards the lookahead reference point
  local alpha = atan2(y_ref - y_rbt, x_ref - x_rbt)
  alpha = alpha - th_rbt
  -- kappa is curvature (inverse of the radius of curvature)
  local two_sa = 2 * sin(alpha)
  local kappa = two_sa / lookahead
  local radius_of_curvature = lookahead / two_sa
  -- Returns the inverse curvature and radius of curvature
  return {
    alpha = alpha,
    kappa = kappa,
    radius_of_curvature = radius_of_curvature,
  }
end
lib.get_inverse_curvature = get_inverse_curvature

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
