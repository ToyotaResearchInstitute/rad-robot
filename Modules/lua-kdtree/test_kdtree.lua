#!/usr/bin/env lua5.1
local unpack = unpack or require'table'.unpack
require'math'.randomseed(1)

local kdtree = require'kdtree'
local kd = assert(kdtree.create(3))
print(kd)

local threshold = 0.5

-- assert(kd:insert{1,2,3})
-- assert(kd:insert{4,5,6})
-- assert(kd:insert{7,8,9})
-- local set = assert(kd:nearest{5, 6, 7})

local pt0 = {math.random(), math.random(), math.random()}
local points = {}
for i=1,1000 do
  local pt = {math.random(), math.random(), math.random()}
  local d = 0
  for j,v in ipairs(pt) do d = d + math.pow(v-pt0[j], 2) end
  d = math.sqrt(d)
  pt.d = d
  if i<=25 then
  print(string.format("%d | %.3f | {%s}",
                      i, pt.d, table.concat(pt, ", ")))
  end
  kd:insert(pt, i)
  table.insert(points, pt)
end
table.sort(points, function(a, b) return a.d<b.d end)
for i, v in ipairs(points) do
  if v.d>threshold then break end
  if i>25 then break end
  print(string.format("%3d: [d=%.3f], {%s}",
                      i, v.d, table.concat(v, ", ")))
end

print("KD tree size:", kd:size())

print("Finding near", unpack(pt0))
local set = assert(kd:nearest(pt0))
print("Nearest", set)
assert(type(set)=='table', "Bad output format")
assert(#set==1, "Should have only the closest")
local res = unpack(set)
print(string.format("Result [user=%d] {%.3f, %.3f, %.3f}", res.user, unpack(res)))

set = assert(kd:nearest(pt0, threshold))
print("Nearest", set)
assert(type(set)=='table', "Bad output format")
for i, v in ipairs(set) do
  print(i, unpack(v))
end

print("Clearing tree...")
assert(kd:clear())
print("KD tree size:", kd:size())
print(kd)
