#!/usr/bin/env luajit
local ff = require'fileformats'

local pts = {
  {1,2,3},{4,5,6},{7,8,9},{10,11,12},{13,14,15}
}
assert(ff.save_ply("test.ply", pts))

local pts1 = assert(ff.load_ply("test.ply", nil, true))

for i, p in ipairs(pts1) do
  print("Points "..i, unpack(p))
end
