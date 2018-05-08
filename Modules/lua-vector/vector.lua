local vector  = {}
local mt      = {}

local sqrt = require'math'.sqrt
local pow = require'math'.pow

function vector.ones(n)
  local t = {}
  for i = 1,(n or 1) do t[i] = 1 end
  return setmetatable(t, mt)
end

local function zeros(n)
  if type(n) ~= 'number' then
    return setmetatable({}, mt)
  end
  local t = {}
  for i = 1, n do t[i] = 0 end
  return setmetatable(t, mt)
end
vector.zeros = zeros

local function new(t)
  local tp = type(t)
  if tp=='number' then
    return zeros(t)
  end
  return setmetatable(tp=='table' and t or {}, mt)
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

local function sub(self, v2)
  local tbl = {}
  -- for i = 1, #self do tbl[i] = self[i] - v2[i] end
  for i,v in ipairs(self) do tbl[i] = v - v2[i] end
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
  if type(v) == "number" then
    return mulnum(self, v)
  elseif type(self)=='number' then
    return mulnum(v, self)
  else
    return dot(self, v)
  end
end
vector.mul = mul

local function unm(v1)
  return mulnum(v1, -1)
end

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
function vector.mean(t, w)
  local s = zeros(#t[1])
  if type(w)=='table' then
    for i,v in ipairs(t) do
      iadd(s, mulnum(v, w[i]))
    end
  else
    for i=1,#t do iadd(s, t[i]) end
    idivnum(s, #t)
  end
  return setmetatable(s, mt)
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
-- Metatables for pose vectors
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
    p[1], p[2], p[3]*180/math.pi
  )
end

function vector.pose(t)
  if type(t)=='table' and #t>=3 then
    -- good pose
    return setmetatable(t, mt_pose)
  end
  return setmetatable({0,0,0}, mt_pose)
end

-- Pose vector
mt_pose.__add = add
mt_pose.__sub = sub
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

return vector
