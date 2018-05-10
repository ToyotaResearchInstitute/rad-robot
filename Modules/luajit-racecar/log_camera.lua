#!/usr/bin/env luajit
local flags = require'racecar'.parse_arg(arg)
local devname = flags[1] or '/dev/video0'

local time = require'unix'.time
local racecar = require'racecar'
local log_announce = racecar.log_announce
local get_jitter = racecar.get_jitter

local width, height = 1344, 376
local camera = require'uvc'.init(
  devname, width, height, 'yuyv', 1, 15)
assert(camera)

local jpeg = require'jpeg'
local c_yuyv = jpeg.compressor('yuyv')
c_yuyv:downsampling(0)

local channel = 'camera'
local logger = require'logger'
local log_dir = (os.getenv"RACECAR_HOME" or '.').."/logs"
local log = flags.log~=0 and assert(logger.new(channel, log_dir))

local function exit()
  if log then log:close() end
  camera:close()
  return 0
end
racecar.handle_shutdown(exit)

local t_debug = time()
local n = 0
while racecar.running do
  local img, sz = camera:get_image(-1)
  local t = time()
  if img then
    local img_jpg = c_yuyv:compress(img, sz, width, height)
    local obj = {
      t = t, jpg = img_jpg
    }
    log_announce(log, obj, channel)
    n = n + 1
  end
  local dt_debug = t - t_debug
  if dt_debug > 1 then
    io.stdout:write(string.format(
      "%3d images @ %f Hz\n", n, n/dt_debug))
    t_debug = t
    n = 0
  end
end
exit()
