#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
local logger = require'logger'
assert(fname:match(logger.iso8601_match), "Bad log datestamp")
local fname_ply = arg[2] or "test.ply"

local xyz_cb = require'velodyne_lidar'.xyz_cb
local function it()
  local t0
  for str, ch, t_us, _ in logger.play(fname, true) do
    if ch=='velodyne' then
      if not t0 then t0 = t_us end
      local dt = tonumber(t_us - t0) / 1e6
      if dt>0.1 then return end
      local obj = logger.decode(str)
      xyz_cb(obj.dist, obj.azi, coroutine.yield)
    end
  end
  return
end

-- Save a ply file
local ff = require'fileformats'
ff.save_ply(fname_ply, coroutine.wrap(it), false)

-- Should load a ply file