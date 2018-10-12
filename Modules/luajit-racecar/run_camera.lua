#!/usr/bin/env luajit
local flags = require'racecar'.parse_arg(arg)
local devname = flags.uvc or flags[1] or '/dev/video0'

local time = require'unix'.time
local racecar = require'racecar'
racecar.init()
local log_announce = racecar.log_announce
-- local get_jitter = racecar.get_jitter

-- local width, height = 1344, 376
local width, height = 320, 240
local camera = require'uvc'.init(
  devname, width, height, 'yuyv', 1, 15)
assert(camera)
local fd_cam = camera:descriptor()

local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor('yuyv')
c_yuyv:downsampling(0)

local channel = 'camera'
local logger = require'logger'
local log = flags.log~=0 and assert(logger.new(channel, racecar.ROBOT_HOME.."/logs"))


local function exit()
  if log then log:close() end
  camera:close()
  return 0
end
racecar.handle_shutdown(exit)

local function update_read(e)
  local img, sz = camera:get_image()
  local t = time()
  if not img then return end
  local img_jpg = c_yuyv:compress(img, sz, width, height)
  local obj = {
    t = t, jpg = img_jpg
  }
  log_announce(log, obj, channel)
end
exit()

-- Listen at 100Hz
local cb_tbl = {
  houston = function() end
}
local fd_updates = {
  [fd_cam] = update_read
}
racecar.listen{
  channel_callbacks = cb_tbl,
  fd_updates = fd_updates,
  -- loop_rate = 30,
  -- loop_fn = cb_loop
}
