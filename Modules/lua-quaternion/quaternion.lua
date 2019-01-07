local quaternion = {}

local sin = require'math'.sin
local cos = require'math'.cos
local acos = require'math'.acos
local asin = require'math'.asin
local atan2 = require'math'.atan2
local abs = require'math'.abs
local sqrt = require'math'.sqrt
local pow = require'math'.pow
local PI = require'math'.pi
local TWO_PI = 2 * PI

local unpack = unpack or require'table'.unpack

local eps = 1e-9

local function vnorm(q)
  if #q==3 then
    return sqrt(pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2))
  elseif #q==4 then
    return sqrt(pow(q[1], 2) + pow(q[2], 2) + pow(q[3], 2) + pow(q[4], 2))
  else
    return false, "Bad q norm"
  end
end

local mt = {}

local function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  local b = a % TWO_PI
  return b >= PI and (b - TWO_PI) or b
end

local function cross(v1, v2)
  return {
  ( (v1[2] * v2[3]) - (v1[3] * v2[2]) ),
  - ( (v1[1] * v2[3]) - (v1[3] * v2[1]) ),
  ( (v1[1] * v2[2]) - (v1[2] * v2[1]) ),
  }
end

local function conjugate(q)
  return setmetatable({
    q[1], -1*q[2], -1*q[3], -1*q[4],
  }, mt)
end
quaternion.conjugate = conjugate

-- Make a unit quaternion, or normalize one
-- Assumes that q is already a quaternion
local function unit(q)
  if type(q)~='table' then return setmetatable({1,0,0,0}, mt) end
  if #q~=4 then return false, "Bad q" end
  local wNorm = vnorm(q)
  if wNorm < eps then
    return setmetatable({1,0,0,0}, mt)
  end
  return setmetatable({
      q[1] / wNorm, q[2] / wNorm, q[3] / wNorm, q[4] / wNorm}, mt)
end
quaternion.unit = unit

-- New quaternion from a 3 element rotation vector
local function from_rotation_vector(t)
  -- Grab the norm of the rotation vector
  local wNorm = vnorm(t)
  local scale = wNorm > eps and (sin(wNorm / 2) / wNorm) or (0.5 - wNorm)
  local q = unit{
    cos(wNorm / 2),
    scale * t[1],
    scale * t[2],
    scale * t[3],
  }
  return setmetatable(q, mt)
end
quaternion.from_rotation_vector = from_rotation_vector

-- Return a rotation vector, which is the logarithm
-- TODO: mod_angle of acos(q1)
local function qlog(q)
  local q1, q2, q3, q4 = unpack(unit(q))
  local mag = sqrt(q1*q1 + q2*q2 + q3*q3)
  -- local mag = sqrt(pow(q1, 2) + pow(q2, 2) + pow(q3, 2))
  -- local mag = vnorm{q2, q3, q4}
  -- local alphaW = 2 * asin(mag)
  -- local factor = alphaW / mag
  local factor = (mag > eps) and (2 * asin(mag) / mag) or (2 + mag)
  return {factor * q2, factor * q3, factor*q4}
end
quaternion.log = qlog

local function from_angle_axis(angle, axis)
  local axNorm = vnorm(axis)
  if axNorm==0 then return unit() end
  local s = sin(angle/2) / axNorm
  return setmetatable({
    cos(angle / 2),
    s * axis[1],
    s * axis[2],
    s * axis[3],
  }, mt)
end
quaternion.from_angle_axis = from_angle_axis

function quaternion.angle_axis(q)
  local angle = 2 * acos(q[1])
  if angle > math.pi then angle = angle - 2 * math.pi end
  -- Avoid the divide by zero scenario
  if angle < eps then
    return 0, {1, 0, 0}
  end
  local sa = sin(angle / 2)
  return angle, {q[2]/sa, q[3]/sa, q[4]/sa}
end

-- Get the angle between two quaternions
local function difference(q0,q1)
  local angle0, axis0 = quaternion.angle_axis(
    conjugate(q0) * q1
  )
  local angle1, axis1 = quaternion.angle_axis(
    conjugate(q0) * -q1
  )
  -- Check the other direction
  if abs(angle1) < abs(angle0) then
    return angle1, axis1
  end
  return angle0, axis0
end
quaternion.difference = difference

-- For the clicking of the object to pickup
-- dipole must be normalized, first
function quaternion.from_dipole( dipole )
  local z_axis = {0, 0, 1}
--  local axis   = cross(dipole, z_axis)
  local axis   = cross(z_axis, dipole)
  local angle  = acos(dipole[1] * z_axis[1] + dipole[2] * z_axis[2] + dipole[3] * z_axis[3])
  return quaternion.from_angle_axis(angle, axis)
end

-- https://en.wikipedia.org/wiki/Slerp
local function slerp(q0,q1,t)
  t = t or .5
  ----[[
  local q0_prime = conjugate(q0)
  local prodQuat = q0_prime * q1
  local angle, axis = quaternion.angle_axis(prodQuat)
  --]]
  --[[
  local angle, axis = diff(q0,q1)
  --]]
  local quadT = quaternion.from_angle_axis(angle * t,axis)
  return q0 * quadT
end
quaternion.slerp = slerp

-- https://theory.org/software/qfa/writeup/node12.html
-- s0, s1: inner quadrangle points
function quaternion.squad(q0, q1, t, s0, s1)
  return slerp(
    slerp(q0, q1, t),
    slerp(s0, s1, t),
    2 * t * (1 - t)
  )
end

-- Makes no sense for quaternions...
--[[
-- Metatable methods are local
local function add(q1, q2)
  local q = {}
  for i,v in ipairs(q1) do q[i] = v + q2[i] end
  return setmetatable(q, mt)
end
--]]

--http://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/
local function negate(q)
  return setmetatable({
    -q[1],-q[2],-q[3],-q[4]
  }, mt)
end

local function mul(q1, q2)
  local a1, b1, c1, d1 = unpack(q1)
  local a2, b2, c2, d2 = unpack(q2)
  return unit{
    a1*a2 - b1*b2 - c1*c2 - d1*d2,
    a1*b2 + b1*a2 + c1*d2 - d1*c2,
    a1*c2 - b1*d2 + c1*a2 + d1*b2,
    a1*d2 + b1*c2 - c1*b2 + d1*a2
  }
end
quaternion.mul = mul

-- Rotate a vector
function quaternion.rotate(q, v)
  return {unpack(mul(q, mul({0, unpack(v)}, conjugate(q))), 2)}
end
function quaternion.inv_rotate(q, v)
  return {unpack(mul(conjugate(q), mul({0, unpack(v)}, q)), 2)}
end

-- TODO: Add a formatting option to replace %g?
local function tostring(q)
  return string.format("{%g, %g, %g, %g}", unpack(q))
end

-- Return the Roll/Pitch/Yaw of this quaternion
-- http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
function quaternion.to_rpy(q)
  local q1, q2, q3, q4 = unpack(q)
  return {
    mod_angle(atan2(2*(q1*q2+q3*q4), 1-2*(q2*q2+q3*q3) )),
    mod_angle(asin(2*(q1*q3-q4*q2))),
    mod_angle(atan2(2*(q1*q4+q2*q3), 1-2*(q3*q3+q4*q4))),
  }
end

function quaternion.from_rpy(roll, pitch, yaw)
  local cy = cos(yaw * 0.5)
  local sy = sin(yaw * 0.5)
  local cr = cos(roll * 0.5)
  local sr = sin(roll * 0.5)
  local cp = cos(pitch * 0.5)
  local sp = sin(pitch * 0.5)

  return unit{
    cy * cr * cp + sy * sr * sp,
    cy * sr * cp - sy * cr * sp,
    cy * cr * sp + sy * sr * cp,
    sy * cr * cp - cy * sr * sp
    }
end

-- Set the metatable values
--mt.__add = add
mt.__sub = difference
mt.__mul = mul
mt.__unm = negate
mt.__tostring = tostring


-- Take the average of a set of quaternions
-- TODO: Remove the torch dependence
----[[
local has_vector, vector = pcall(require, 'vector')
if has_vector then
  local vsub = vector.sub
  quaternion.mean = function(qs, ws)
    local qIter = qs[1]
    local iter = 0
    local err = {}
    repeat
      iter = iter + 1
      for i, qi in ipairs(qs) do
        local eQ = qi * conjugate(qIter)
        err[i] = qlog(eQ)
      end
      local errMean = vector.mean(err, ws)
      local qErrMean = from_rotation_vector(errMean)
      local qIterNext = qErrMean * qIter
      -- Compare the rotation vectors
      local diff = vsub(qlog(qIterNext), qlog(qIter))
      local done = (vnorm(diff) < 1e-6) or (iter > 1e3)
      if not done then qIter = qIterNext end
    until done
    return qIter, err
  end
end
--]]

local has_lapack, lapack = pcall(require, 'lapack')
-- TODO: Remove the torch dependence
if has_lapack then
  local matrix = require'matrix'
  -- https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions
  function quaternion.mean2(qs, ws)
    if type(ws)=='table' then
      local qs1 = {}
      for i,q in ipairs(qs) do qs1[i] = vector.mulnum(q, ws[i]) end
      qs = qs1
    end
    qs = matrix:new(qs)
    -- TODO: dsyrk
    -- https://stackoverflow.com/questions/47013581/blas-matrix-by-matrix-transpose-multiply
    local _, evecs = lapack.eigs(qs:transpose() * qs)
    -- Eigvectors along columns...
    evecs = matrix.transpose(evecs)
    return unit(evecs[1])
  end
end

return quaternion
