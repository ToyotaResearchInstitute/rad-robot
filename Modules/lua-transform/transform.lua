local lib = {}
local asin = require'math'.asin
local atan2 = require'math'.atan2 or require'math'.atan
local cos = require'math'.cos
local sin = require'math'.sin
local PI = require'math'.pi
local sqrt = require'math'.sqrt
local rad2deg = require'math'.deg

local unpack = unpack or require'table'.unpack
local nan = 0 / 0

-- TODO: Unroll all loops

local mt = {
  __tostring = function(tr)
    local pr = {}
    for i=1,4 do
      local row = {}
      for j=1,4 do
        table.insert(row, string.format('%6.3f', tr[i][j] or nan))
      end
      local c = table.concat(row, ', ')
      table.insert(pr, string.format('[%s]',c))
    end
    return table.concat(pr, '\n')
  end
}

local TWO_PI = 2 * PI
local function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  local b = a % TWO_PI
  return b >= PI and (b - TWO_PI) or b
end
lib.mod_angle = mod_angle

local function rot2D(x, y, th)
  local c, s = cos(th), sin(th)
  return x * c - y * s, x * s + y * c
end
lib.rot2D = rot2D

local function tf2D(x, y, th, tx, ty)
  local x1, y1 = rot2D(x, y, th)
  return x1 + tx, y1 + ty
end
lib.tf2D = tf2D

local function tf2D_inv(x, y, th, tx, ty)
  return rot2D(x - tx, y - ty, -th)
end
lib.tf2D_inv = tf2D_inv

local function rot2Dcs(x, y, c, s)
  return x * c - y * s, x * s + y * c
end
lib.rot2Dcs = rot2Dcs
local function tf2Dcs(x, y, c, s, tx, ty)
  local x1, y1 = rot2D(x, y, c, s)
  return x1 + tx, y1 + ty
end
lib.tf2Dcs = tf2Dcs
local function tf2Dcs_inv(x, y, c, s, tx, ty)
  return rot2D(x - tx, y - ty, c, -s)
end
lib.tf2Dcs_inv = tf2Dcs_inv

function lib.inv(a)
  local p = {a[1][4], a[2][4], a[3][4]}
  local r = {
    {a[1][1], a[2][1], a[3][1]},
    {a[1][2], a[2][2], a[3][2]},
    {a[1][3], a[2][3], a[3][3]}
  }
  return setmetatable({
    {r[1][1], r[1][2], r[1][3], -(r[1][1]*p[1]+r[1][2]*p[2]+r[1][3]*p[3])},
    {r[2][1], r[2][2], r[2][3], -(r[2][1]*p[1]+r[2][2]*p[2]+r[2][3]*p[3])},
    {r[3][1], r[3][2], r[3][3], -(r[3][1]*p[1]+r[3][2]*p[2]+r[3][3]*p[3])},
    {0,0,0,1}
  }, mt)
end

function lib.eye()
  return setmetatable({
    {1, 0, 0, 0},
    {0, 1, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  }, mt)
end

function lib.rotX(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
    {1, 0, 0, 0},
    {0, ca, -sa, 0},
    {0, sa, ca, 0},
    {0, 0, 0, 1}
  }, mt)
end

function lib.rotY(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
    {ca, 0, sa, 0},
    {0, 1, 0, 0},
    {-sa, 0, ca, 0},
    {0, 0, 0, 1}
  }, mt)
end

function lib.rotZ(a)
  local ca = cos(a)
  local sa = sin(a)
  return setmetatable({
    {ca, -sa, 0, 0},
    {sa, ca, 0, 0},
    {0, 0, 1, 0},
    {0, 0, 0, 1}
  }, mt)
end

function lib.trans(dx, dy, dz)
  return setmetatable({
    {1, 0, 0, dx},
    {0, 1, 0, dy},
    {0, 0, 1, dz},
    {0, 0, 0, 1}
  }, mt)
end

-- Mutate a matrix
function lib.rotateXdot(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local ty = t[i][2]
    local tz = t[i][3]
    t[i][1] = 0
    t[i][2] = -sa*ty + ca*tz
    t[i][3] = -ca*ty - sa*tz
    t[i][4] = 0
  end
  return t
end

function lib.rotateYdot(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local tx = t[i][1]
    local tz = t[i][3]
    t[i][1] = -sa*tx - ca*tz
    t[i][2] = 0
    t[i][3] = ca*tx - sa*tz
    t[i][4] = 0
  end
  return t
end

function lib.rotateZdot(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local tx = t[i][1]
    local ty = t[i][2]
    t[i][1] = -sa*tx + ca*ty
    t[i][2] = -ca*tx - sa*ty
    t[i][3] = 0
    t[i][4] = 0
  end
  return t
end

function lib.rotateX(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local ty = t[i][2]
    local tz = t[i][3]
    t[i][2] = ca*ty + sa*tz
    t[i][3] = -sa*ty + ca*tz
  end
  return t
end

function lib.rotateY(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local tx = t[i][1]
    local tz = t[i][3]
    t[i][1] = ca*tx - sa*tz
    t[i][3] = sa*tx + ca*tz
  end
  return t
end

function lib.rotateZ(t, a)
  local ca = cos(a)
  local sa = sin(a)
  for i=1,3 do
    local tx = t[i][1]
    local ty = t[i][2]
    t[i][1] = ca*tx + sa*ty
    t[i][2] = -sa*tx + ca*ty
  end
  return t
end

function lib.translate(t, px, py, pz)
  t[1][4] = t[1][4] + t[1][1]*px + t[1][2]*py + t[1][3]*pz
  t[2][4] = t[2][4] + t[2][1]*px + t[2][2]*py + t[2][3]*pz
  t[3][4] = t[3][4] + t[3][1]*px + t[3][2]*py + t[3][3]*pz
  return t
end

-- End mutations

-- Recovering Euler Angles
-- Good resource: http://www.vectoralgebra.info/eulermatrix.html
function lib.to_zyz(t)
  -- Modelling and Control of Robot Manipulators, pg. 30
  -- Lorenzo Sciavicco and Bruno Siciliano
  return {
    atan2(t[2][3],t[1][3]), -- Z (phi)
    atan2(sqrt( t[1][3]^2 + t[2][3]^2),t[3][3]), -- Y (theta)
    atan2(t[3][2],-t[3][1]) -- Z' (psi)
  }
end

-- RPY is XYZ convention
function lib.to_rpy(t)
  -- http://planning.cs.uiuc.edu/node103.html
  -- returns [roll, pitch, yaw] vector
  return {
    atan2(t[3][2],t[3][3]), --Roll
    atan2(-t[3][1],sqrt( t[3][2]^2 + t[3][3]^2)), -- Pitch
    atan2(t[2][1],t[1][1]) -- Yaw
  }
end

function lib.position6D(tr)
  return {
    tr[1][4], tr[2][4], tr[3][4],
    atan2(tr[3][2],tr[3][3]),
    -asin(tr[3][1]),
    atan2(tr[2][1],tr[1][1])
  }
end
function lib.string6D(tr)
  return string.format('%.2f %.2f %.2f | %.2f %.2f %.2f',
    tr[1][4],tr[2][4],tr[3][4],
    rad2deg(atan2(tr[3][2],tr[3][3])),
    rad2deg(-asin(tr[3][1])),
    rad2deg(atan2(tr[2][1],tr[1][1])))
end

function lib.position(tr)
  return {tr[1][4],tr[2][4],tr[3][4]}
end

function lib.position4(tr)
  return {tr[1][4],tr[2][4],tr[3][4],tr[4][4]}
end

function lib.from_quaternion(q, pos)
  -- if type(q)~='table' then
  --   return false, "Bad quaternion"
  -- end
  local q1, q2, q3, q4 = unpack(q)
  local x, y, z
  if type(pos) == 'table' then
    x, y, z = unpack(pos)
  else
    x, y, z = 0, 0, 0
  end
  return setmetatable({
    {
      1 - 2 * q3 * q3 - 2 * q4 * q4,
      2 * q2 * q3 - 2 * q4 * q1,
      2 * q2 * q4 + 2 * q3 * q1,
      x
    },
    {
      2 * q2 * q3 + 2 * q4 * q1,
      1 - 2 * q2 * q2 - 2 * q4 * q4,
      2 * q3 * q4 - 2 * q2 * q1,
      y
    },
    {
      2 * q2 * q4 - 2 * q3 * q1,
      2 * q3 * q4 + 2 * q2 * q1,
      1 - 2 * q2 * q2 - 2 * q3 * q3,
      z
    },
    {0,0,0,1}
  }, mt)
end

-- Rotation Matrix to quaternion
function lib.to_quaternion(t)
  -- Compute the trace
  local a, b, c = t[1][1], t[2][2], t[3][3]
  local tr = a + b + c
  if tr > 0 then
    local S = 2 * sqrt(tr + 1)
    return{
      0.25 * S,
      (t[3][2] - t[2][3]) / S,
      (t[1][3] - t[3][1]) / S,
      (t[2][1] - t[1][2]) / S,
    }
  elseif a > b and a > c then
    local S = 2 * sqrt(1 + a - b - c)
    return {
      (t[3][2] - t[2][3]) / S,
      0.25 * S,
      (t[1][2] + t[2][1]) / S,
      (t[1][3] + t[3][1]) / S
    }
  elseif b > c then
    local S = 2 * sqrt(1.0 + b - a - c)
    return {
      (t[1][3] - t[3][1]) / S,
      (t[1][2] + t[2][1]) / S,
      0.25 * S,
      (t[2][3] + t[3][2]) / S,
    }
  else
    local S = 2 * sqrt(1.0 + c - a - b)
    return {
      (t[2][1] - t[1][2]) / S,
      (t[1][3] + t[3][1]) / S,
      (t[2][3] + t[3][2]) / S,
      0.25 * S,
    }
  end
end

function lib.transform6D(p)
  local cwx = cos(p[4])
  local swx = sin(p[4])
  local cwy = cos(p[5])
  local swy = sin(p[5])
  local cwz = cos(p[6])
  local swz = sin(p[6])
  return setmetatable({
    {cwy*cwz, swx*swy*cwz-cwx*swz, cwx*swy*cwz+swx*swz, p[1]},
    {cwy*swz, swx*swy*swz+cwx*cwz, cwx*swy*swz-swx*cwz, p[2]},
    {-swy, swx*cwy, cwx*cwy, p[3]},
    {0,0,0,1},
    }, mt)
end

-- http://planning.cs.uiuc.edu/node102.html
function lib.from_rpy_trans(rpy, trans)
  local gamma, beta, alpha = unpack(rpy)
  trans = trans or {0,0,0}
  return setmetatable({
    {cos(alpha) * cos(beta),
      cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma),
      cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
      trans[1]},
    {sin(alpha) * cos(beta),
      sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma),
      sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
      trans[2]},
    {-sin(beta), cos(beta) * sin(gamma), cos(beta) * cos(gamma), trans[3]},
    {0, 0, 0, 1}
  }, mt)
end

-- Multiply vector
local function mv(m, p0)
  local x, y, z, s = unpack(p0)
  s = s or 1
  local p = {}
  for i = 1,3 do
    local a,b,c,d = unpack(m[i])
    p[i] = a * x + b * y + c * z + d * s
  end
  if #p0 == 4 then p[4] = s * m[4][4] end
  return p
end

-- Assume t1 is proper 4x4 tf matrix
function lib.mul(t1, t2)
  if type(t2[1]) == "number" then
    -- Matrix * Vector
    return mv(t1, t2)
  end
  -- Matrix * Matrix
  local t = {{},{},{},{}}
  -- Matrix * Matrix
  for i = 1,4 do
    local a, b, c, d = unpack(t1[i])
    for j = 1,4 do
      t[i][j] = a * t2[1][j]
              + b * t2[2][j]
              + c * t2[3][j]
              + d * t2[4][j]
    end
  end
  return setmetatable(t, mt)
end
mt.__mul = lib.mul

-- TODO: Inline, overwriting the input?
function lib.batch_mul(m, pts, inline)
  local xs0, ys0, zs0 = unpack(pts)
  local xs, ys, zs
  if inline then
    xs, ys, zs = xs0, ys0, zs0
  else
    xs, ys, zs = {}, {}, {}
  end
  for i=1,#xs0 do
    local p = mv(m, {xs0[i], ys0[i], zs0[i]})
    xs[i], ys[i], zs[i] = unpack(p)
  end
  return {xs, ys, zs}
end

-- Copy
function lib.copy(tt, t0)
  if type(tt)=='table' and not t0 then
    -- Copy the table
    return setmetatable({
      {unpack(tt[1])},
      {unpack(tt[2])},
      {unpack(tt[3])},
      {unpack(tt[4])},
    }, mt)
  end
  local t = t0 or setmetatable({{},{},{},{}}, mt)
  for i=1,3 do
    for j=1,4 do
      t[i][j] = tt[i][j]
    end
  end
  -- copy a tensor
  return t
end

function lib.from_flat(flat)
  return setmetatable({
    {unpack(flat, 1, 4)},
    {unpack(flat, 5, 8)},
    {unpack(flat, 9, 12)},
    {unpack(flat, 13, 16)}
  }, mt)
end
-- Do it unsafe; assume a table
function lib.flatten(t)
  return {
    t[1][1], t[1][2], t[1][3], t[1][4],
    t[2][1], t[2][2], t[2][3], t[2][4],
    t[3][1], t[3][2], t[3][3], t[3][4],
    t[4][1], t[4][2], t[4][3], t[4][4]
  }
end

-- Do it unsafe; assume a table
function lib.new(tt)
  return setmetatable(tt, mt)
end

return lib
