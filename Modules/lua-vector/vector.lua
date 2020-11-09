local lib  = {}
local mt      = {}

local deg2rad = require'math'.rad
local rad2deg = require'math'.deg
local max = require'math'.max
local min = require'math'.min
local log = require'math'.log
local PI = require'math'.pi
local random = require'math'.random
local sqrt = require'math'.sqrt
--
local tmove = require'table'.move
local tremove = require'table'.remove
local unpack = unpack or require'table'.unpack
-- https://en.wikipedia.org/wiki/Machine_epsilon
local EPSILON = 2.22044604925031308e-16

-- Use the LuaJIT extension, when possible
local has_tnew, tnew = pcall(require, 'table.new')
if not has_tnew then
  tnew = function() return {} end
end

function lib.ones(n)
  n = n or 1
  local t = tnew(n, 0)
  for i = 1,n do t[i] = 1 end
  return setmetatable(t, mt)
end

local function zeros(n)
  n = n or 1
  local t = tnew(n, 0)
  for i = 1,n do t[i] = 0 end
  return setmetatable(t, mt)
end
lib.zeros = zeros

function lib.random(n, a, b)
  n = n or 1
  local t = tnew(n, 0)
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
  return mu + (x1 * w) * std, mu + (x2 * w) * std
end
function lib.randn(n, std, mu)
  n = n or 1
  local t = tnew(n, 0)
  for i = 1, n, 2 do
    t[i], t[i+1] = randn(std, mu)
  end
  -- Went two by two
  if #t > n then tremove(t) end
  return setmetatable(t, mt)
end

local function new(t)
  if type(t)=='number' then
    return zeros(t)
  elseif type(t) == 'table' then
    return setmetatable(t, mt)
  end
  return setmetatable({}, mt)
end
lib.new = new

local function copy(t_source, t_dest)
  --return setmetatable(table.move(a1, 1, #t_source, 1, t_dest), mt)
  ----[[
  t_dest = t_dest or {}
  for i=1,#t_source do t_dest[i] = t_source[i] end
  return setmetatable(t_dest, mt)
  --]]
end
lib.copy = copy

function lib.count(start, n)
  n = n or 1
  local t = tnew(n, 0)
  for i = 1,n do t[i] = start+i-1 end
  return setmetatable(t, mt)
end

function lib.slice(v1, istart, iend)
  return setmetatable(tmove(a1, istart, iend, 1), mt)
  -- local v = {}
  -- istart = istart or 1
  -- iend = iend or #v1
  -- if istart==iend then return v1[istart] end
  -- for i = 1,iend-istart+1 do
  --   v[i] = v1[istart+i-1] or (0 / 0)
  -- end
  -- return setmetatable(v, mt)
end

function lib.contains(v1, item)
  for i=1,#v1 do
    if v1[i]==item then return true end
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
lib.iadd = iadd

local function norm_sq(v1)
  local s = 0
  for i = 1, #v1 do s = s + v1[i]^2 end
  return s
end
lib.norm_sq = norm_sq

local function norm(v1)
  return sqrt(norm_sq(v1))
end
lib.norm = norm

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
lib.sum = sum
-- Recursive sum
local function rsum(v, idx, initial)
  idx = idx or 1
  initial = initial or 0
  if idx > #v then return initial end
  return rsum(v, idx + 1, v[idx] + initial)
end
lib.rsum = rsum

local function sub(v1, v2)
  local tbl = {}
  if type(v2)=='number' then
    for i = 1, #v1 do tbl[i] = v1[i] - v2 end
  elseif type(v1) == 'number' then
    for i = 1, #v2 do tbl[i] = v1 - v2[i] end
  else
    for i = 1, #v1 do tbl[i] = v1[i] - v2[i] end
  end
  return setmetatable(tbl, mt)
end
lib.sub = sub

local function mulnum(v1, a)
  local v = {}
  for i = 1, #v1 do v[i] = a * v1[i] end
  return setmetatable(v, mt)
end
lib.mulnum = mulnum

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
lib.idivnum = idivnum

local function dot(self, v2)
  local s = 0
  for i = 1, #self do s = s + self[i] * v2[i] end
  return s
end
lib.dot = dot

local function mul(self, v)
  if type(self)=='number' then
    return mulnum(v, self)
  elseif type(v) == 'number' then
    return mulnum(self, v)
  else
    return dot(self, v)
  end
end
lib.mul = mul

local function unm(self)
  local out = {}
  for i = 1, #self do out[i] = -1 * self[i] end
  return setmetatable(out, mt)
end

-- Multiply matrix and vector
-- TODO: Symmetric (e.g. covariance)
local function mv(m, v)
  local v1 = {}
  for j=1, #m do
    v1[j] = 0
    for i = 1,#v do
      v1[j] = v1[j] + m[j][i] * v[i]
    end
  end
  return setmetatable(v1, mt)
end
lib.mv = mv

function lib.unit(v1)
  local m = norm(v1)
  return (m > EPSILON) and divnum(v1, m) or false
end

local function eq(self, v2)
  if #self~=#v2 then return false end
  for i=1,#self do
    if self[i]~=v2[i] then return false end
  end
  return true
end
lib.eq = eq

local function div(v1, v2)
  if type(v2) == "number" then
    return divnum(v1, v2)
  else
    -- pointwise
    local v = {}
    for i in #v1 do v[i] = v1[i] / v2[i] end
    return setmetatable(v, mt)
  end
end

-- Ability for a weighted mean of vectors
local function mean(t, w)
  local has_weights = type(w)=='table'
  local n = #t
  local mu = t[1]
  -- Vector of numbers (default)
  if type(mu) == 'table' then
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
lib.mean = mean

-- Calculate the variance and sample variance of a vector of numbers
function lib.variance(t, w, mu0)
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
lib.distance_sq = distance_sq

local function distance(self, v)
  return norm(sub(v, self))
end
lib.distance = distance

-- Project vector self onto vector v
local function project(self, v)
  local s = dot(self, v) / norm(v)
  return mulnum(v, s)
end
lib.project = project

-- Rejection vector self onto vector v
function lib.reject(self, v)
  return sub(self, project(self, v))
end

local function set(self)
  local n = #self
  local s = {n=n}
  for i=1,n do s[self[i]] = true end
  return s
end
lib.set = set

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
lib.eq_set = eq_set

local function set_eq_set(s1, s2)
  for k1 in pairs(s1) do
    if not s2[k1] then return false end
  end
  for k2 in pairs(s2) do
    if not s1[k2] then return false end
  end
  return true
end
lib.set_eq_set = set_eq_set

-- local function cross(u, v)
--   local k = u[1] * v[2], -1 * u[2] * v[1]

--   return {i, j, k}
-- end
-- lib.cross = cross

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
    p[1], p[2], rad2deg(p[3])
  )
end

function lib.pose(t)
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
lib.mod_angle = mod_angle

local function relative_se2(b, a)
  -- b in frame of a
  return setmetatable({b[1] - a[1], b[2] - a[2], mod_angle(b[3] - a[3])},
    mt_pose)
end

-- TODO: Process noise and measurement noise functions
local function randn_pose(std_pose, mu_pose)
  -- Limit the noise
  local dx_noise_max = 0.01
  local dy_noise_max = 0.01
  local da_noise_max = deg2rad(1)
  -- Add some noise (randn gives two putputs)
  local dx_noise = randn(std_pose[1], 0)
  local dy_noise = randn(std_pose[2], 0)
  local da_noise = randn(std_pose[3], 0)
  -- Clamp the noise
  dx_noise = min(max(-dx_noise_max, dx_noise), dx_noise_max)
  dy_noise = min(max(-dy_noise_max, dy_noise), dy_noise_max)
  da_noise = min(max(-da_noise_max, da_noise), da_noise_max)
  -- Return a new pose
  local pose_w_noise = {
    mu_pose[1] + dx_noise,
    mu_pose[2] + dy_noise,
    mu_pose[3] + da_noise
  }
  return pose_w_noise
end
lib.randn_pose = randn_pose

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

-- Add (simple) math function mapping support
-- local math = require'math'
-- return setmetatable(lib, {
--   __index = function(t, k)
--     local m = math[k]
--     if type(m) == 'function' then
--       -- Map through a math function
--       return function(tbl)
--         local out = {}
--         for i in #tbl do
--           out[i] = type(tbl[i])=='table' and m(unpack(tbl[i])) or m(tbl[i])
--         end
--         return setmetatable(out, mt)
--       end
--     elseif type(m) == 'number' then
--       -- Populate with a math number
--       return function(n)
--         local out = {}
--         for i=1,n do out[i] = m end
--         return setmetatable(out, mt)
--       end
--     end
--   end
-- })

return lib