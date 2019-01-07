local vector  = {}
local mt      = {}

local math = require'math'
local log = require'math'.log
local PI = require'math'.pi
local pow = require'math'.pow
local random = require'math'.random
local sqrt = require'math'.sqrt
local tremove = require'table'.remove
local unpack = unpack or require'table'.unpack

function vector.ones(n)
  local t = {}
  n = n or 1
  for i = 1,n do t[i] = 1 end
  return setmetatable(t, mt)
end

local function zeros(n)
  local t = {}
  n = n or 1
  for i = 1,n do t[i] = 0 end
  return setmetatable(t, mt)
end
vector.zeros = zeros

function vector.random(n, a, b)
  local t = {}
  n = n or 1
  if b then
    for i = 1, n do t[i] = random(a, b) end
  elseif a then
    for i = 1, n do t[i] = random(a) end
  else
    for i = 1, n do t[i] = random() end
  end
  return setmetatable(t, mt)
end

-- Gaussian distributed data
local function randn(std, mu)
  std = tonumber(std) or 1
  mu = tonumber(mu) or 0
  -- https://www.taygeta.com/random/gaussian.html
  local x1, x2
  local w = 1
  while w >= 1 do
    x1 = 2.0 * random() - 1.0
    x2 = 2.0 * random() - 1.0
    w = x1 * x1 + x2 * x2
  end
  w = sqrt( (-2.0 * log( w ) ) / w )
  -- Two numbers given for free
  local y1 = x1 * w
  local y2 = x2 * w
  return mu + y1 * std, mu + y2 * std
end
function vector.randn(n, std, mu)
  local t = {}
  for i = 1, n, 2 do
    t[i], t[i+1] = randn(std, mu)
  end
  -- Went two by two
  while #t>n do tremove(t) end
  return setmetatable(t, mt)
end

local function new(t)
  local tp = type(t)
  if tp=='number' then
    return zeros(t)
  elseif tp == 'table' then
    return setmetatable(t, mt)
  end
  return setmetatable({}, mt)
end
vector.new = new

local function copy(t, tt)
  tt = tt or {}
  for i=1,#t do tt[i] = t[i] end
  return setmetatable(tt, mt)
end
vector.copy = copy

function vector.count(start, n)
  local t = {}
  n = n or 1
  for i = 1,n do t[i] = start+i-1 end
  return setmetatable(t, mt)
end

function vector.slice(v1, istart, iend)
  local v = {}
  istart = istart or 1
  iend = iend or #v1
  if istart==iend then return v1[istart] end
  for i = 1,iend-istart+1 do
    v[i] = v1[istart+i-1] or (0 / 0)
  end
  return setmetatable(v, mt)
end

function vector.contains(v1, num)
  for i=1,#v1 do
    if v1[i]==num then return true end
  end
  return false
end

local function add(v1, v2)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] + v2[i] end
  return setmetatable(v, mt)
end
-- In-place addition
local function iadd(v1, v2)
  for i = 1, #v1 do v1[i] = v1[i] + v2[i] end
  return v1
end
vector.iadd = iadd

local function norm_sq(v1)
  local s = 0
  for i = 1, #v1 do s = s + pow(v1[i], 2) end
  return s
end
vector.norm_sq = norm_sq

local function norm(v1)
  return sqrt(norm_sq(v1))
end
vector.norm = norm

local function sum(v1, w)
  local s
  if type(w)=='table' then
    s = v1[1] * w[1]
    for i = 2, #v1 do s = s + v1[i] * w[i] end
  else
    s = v1[1]
    -- Must copy, in case only one element
    s = type(s)=='table' and copy(s) or s
    for i = 2, #v1 do s = s + v1[i] end
  end
  return s
end
vector.sum = sum
-- Recursive sum
local function rsum(v, idx, initial)
  idx = idx or 1
  initial = initial or 0
  if idx > #v then return initial end
  return rsum(v, idx + 1, v[idx] + initial)
end
vector.rsum = rsum

local function sub(v1, v2)
  local tbl = {}
  if type(v2)=='number' then
    for i,v in ipairs(v1) do tbl[i] = v - v2 end
  elseif type(v1) == 'number' then
    for i,v in ipairs(v2) do tbl[i] = v1 - v end
  else
    -- for i = 1, #v1 do tbl[i] = v1[i] - v2[i] end
    for i,v in ipairs(v1) do tbl[i] = v - v2[i] end
  end
  return setmetatable(tbl, mt)
end
vector.sub = sub

local function mulnum(v1, a)
  local v = {}
  for i = 1, #v1 do v[i] = a * v1[i] end
  return setmetatable(v, mt)
end
vector.mulnum = mulnum

local function divnum(v1, a)
  local v = {}
  for i = 1, #v1 do v[i] = v1[i] / a end
  return setmetatable(v, mt)
end
-- In-place division
local function idivnum(self, a)
  for i = 1, #self do self[i] = self[i] / a end
  return self
end
vector.idivnum = idivnum

local function dot(self, v2)
  local s = 0
  for i = 1, #self do s = s + self[i] * v2[i] end
  return s
end
vector.dot = dot

local function mul(self, v)
  if type(self)=='number' then
    return mulnum(v, self)
  elseif type(v) == "number" then
    return mulnum(self, v)
  else
    return dot(self, v)
  end
end
vector.mul = mul

local function unm(v1)
  return mulnum(v1, -1)
end

-- Multiply matrix and vector
-- TODO: Symmetric (e.g. covariance)
local function mv(m, v)
  local v1 = {}
  for j=1, #m do
    local mj = m[j]
    v1[j] = 0
    for i = 1,#v do
      v1[j] = v1[j] + mj[i] * v[i]
    end
  end
  return setmetatable(v1, mt)
end
vector.mv = mv

function vector.unit(v1)
  local m = norm(v1)
  return (m > 0) and divnum(v1, m) or zeros(#v1)
end

local function eq(self, v2)
  if #self~=#v2 then return false end
  for i,v in ipairs(self) do
    if v~=v2[i] then return false end
  end
  return true
end
vector.eq = eq

local function div(v1, v2)
  if type(v2) == "number" then
    return divnum(v1, v2)
  else
    -- pointwise
    local v = {}
    for i,val in ipairs(v1) do v[i] = val / v2[i] end
    return setmetatable(v, mt)
  end
end

-- Ability for a weighted mean of vectors
local function mean(t, w)
  local has_weights = type(w)=='table'
  local n = #t
  local mu = t[1]
  local tp = type(mu)
  -- Vector of numbers (default)
  if tp == 'table' then
    mu = copy(mu)
    if has_weights then
      mu = mu * w[1]
      for i=2,#t do iadd(mu, mulnum(t[i], w[i])) end
    else
      for i=2,#t do iadd(mu, divnum(t[i], n)) end
    end
    return setmetatable(mu, mt)
  end
  if has_weights then
    mu = mu * w[1]
    for i=2,n do mu = mu + w[i] * t[i] end
  else
    for i=2,n do mu = mu + t[i] / n end
  end
  return mu
end
vector.mean = mean

-- Calculate the variance and sample variance of a vector of numbers
function vector.variance(t, w, mu0)
  -- Grab the mean: pre-calculated input mu0 as default
  if mu0==false then
    -- Do not shift
    mu0 = 0
  elseif type(mu0)~='number' then
    -- Use a two pass approach
    mu0 = mean(t, w)
  end
  local total = #t
  local mu = 0
  local M2 = 0 -- Aggregate squared distance
  for count=1, #t do
    -- Variance remains the same after pre-calculated shift
    -- This subtraction can make the computation more stable
    local newValue = t[count] - mu0
    -- Find the difference before incremental mean shift
    local delta = newValue - mu
    mu = mu + delta / count
    -- Find the difference after incremental mean shift
    local delta2 = newValue - mu
    M2 = M2 + delta * delta2
  end
  local var = M2 / total
  local sample_var = M2 / (total - 1)
  return var, sample_var
end

-- Distance between points
local function distance_sq(self, v)
  return norm_sq(sub(v, self))
end
vector.distance_sq = distance_sq

local function distance(self, v)
  return norm(sub(v, self))
end
vector.distance = distance

-- Project vector self onto vector v
local function project(self, v)
  local s = dot(self, v) / norm(v)
  return mulnum(v, s)
end
vector.project = project

-- Rejection vector self onto vector v
function vector.reject(self, v)
  return sub(self, project(self, v))
end

local function set(self)
  local n = #self
  local s = {n=n}
  for i=1,n do s[self[i]] = true end
  return s
end
vector.set = set

-- Check if this vector is a permutation of the set s
local function eq_set(self, s)
  local n = #self
  -- Ensure the same number of elements
  if n ~= #s then return false end
  -- Check the hash
  for i=1, n do
    if not s[self[i]] then return false end
  end
  return true
end
vector.eq_set = eq_set

local function set_eq_set(s1, s2)
  for k1 in pairs(s1) do
    if not s2[k1] then return false end
  end
  for k2 in pairs(s2) do
    if not s1[k2] then return false end
  end
  return true
end
vector.set_eq_set = set_eq_set

-- local function cross(u, v)
--   local k = u[1] * v[2], -1 * u[2] * v[1]

--   return {i, j, k}
-- end
-- vector.cross = cross

local function v_tostring(v1, formatstr)
  formatstr = formatstr or "%g"
  local tbl = {}
  for i = 1, #v1 do
    table.insert(tbl, string.format(formatstr,v1[i]))
  end
  return "{"..table.concat(tbl, ', ').."}"
end

----[[
-- Metatables for se2 pose vectors
local mt_pose = {}
-- TODO: Use as a utility pose file, too
local function pose_index(p, idx)
  if idx=='x' then
    return p[1]
  elseif idx=='y' then
    return p[2]
  elseif idx=='a' then
    return p[3]
  end
end

local function pose_newindex(p, idx, val)
  if idx=='x' then
    p[1] = val
  elseif idx=='y' then
    p[2] = val
  elseif idx=='a' then
    p[3] = val
  end
end

local function pose_tostring(p)
  return string.format(
    "{x=%g, y=%g, a=%g degrees}",
    p[1], p[2], math.deg(p[3])
  )
end

function vector.pose(t)
  if type(t)=='table' and #t>=3 then
    -- good pose
    return setmetatable(t, mt_pose)
  end
  return setmetatable({0,0,0}, mt_pose)
end

local TWO_PI = 2 * PI
local function mod_angle(a)
  -- Reduce angle to [-pi, pi)
  local b = a % TWO_PI
  return b >= PI and (b - TWO_PI) or b
end
vector.mod_angle = mod_angle

local function relative_se2(b, a)
  -- b in frame of a
  local ca, sa = cos(a[3]), sin(a[3])
  return setmetatable({b[1] - a[1], b[2] - a[2], mod_angle(b[3] - a[3])}, mt_pose)
end

-- Pose vector
mt_pose.__add = add
mt_pose.__sub = relative_se2 -- TODO: diff in se2
mt_pose.__mul = mul
mt_pose.__div = div
mt_pose.__unm = unm
mt_pose.__index    = pose_index
mt_pose.__newindex = pose_newindex
mt_pose.__tostring = pose_tostring
--]]

-- Regular vector
mt.__eq  = eq
mt.__add = add
mt.__sub = sub
mt.__mul = mul
mt.__div = div
mt.__unm = unm
mt.__tostring = v_tostring
mt.__index = function(t, k) return vector[k] end

-- Add (simple) math function support
return setmetatable(vector, {
  __index = function(t, k)
    local m = math[k]
    if type(m) == 'function' then
      -- Map through a math function
      return function(tbl)
        local out = {}
        for i, v in ipairs(tbl) do
          out[i] = type(v)=='table' and m(unpack(v)) or m(v)
        end
        return setmetatable(out, mt)
      end
    elseif type(m) == 'number' then
      -- Populate with a math number
      return function(n)
        local out = {}
        for i=1,n do out[i] = m end
        return setmetatable(out, mt)
      end
    end
  end
})
