#!/usr/bin/env luajit
local flags = require'racecar'.parse_arg(arg)
local devname = flags.uvc or flags[1]

local uvc = require'uvc'
local time = require'unix'.time
local racecar = require'racecar'
racecar.init()
local log_announce = racecar.log_announce

-- local width, height = 1344, 376
local width, height = 640, 480
local fps = 5
-- local width, height = 320, 240
local fmt = flags.fmt or 'yuyv'
local camera = assert(uvc.init(devname, width, height, fmt, 1, fps))

local c_jpeg
if fmt=='yuyv' then
  local jpeg = require'jpeg'
  c_jpeg = jpeg.compressor'yuyv'
  c_jpeg:downsampling(1)
  -- c_jpeg:downsampling(0)
elseif fmt == 'mjpeg' then
  c_jpeg = nil
  -- local ffi = require'ffi'
  -- c_jpeg = function(ptr, sz) return ffi.string(ptr, sz) end
end

local channel = devname and devname:match("([^/]+%d+)") or 'camera'
local logger = require'logger'
local log = flags.log~=0 and assert(logger.new(channel, racecar.ROBOT_HOME.."/logs"))

local function exit()
  if log then log:close() end
  camera:close()
  return 0
end
-- racecar.handle_shutdown(exit)

local function process_img(img, sz, t)
  local img_jpg = nil
  if c_jpeg then
    img_jpg = c_jpeg:compress(img, sz, width, height)
  elseif fmt == 'mjpeg' then
    img_jpg = img
  end
  local obj = {
    t = t, jpg = img_jpg
  }
  log_announce(log, obj, channel)
end

local function update_read(evt)
  if evt==32 then
    return false, "Bad read"
  end
  local img, sz, cnt = camera:get_image(0, not c_jpeg)
  local t = time()
  -- print("Image", type(img), t, cnt)
  if not img then
    print("Error", sz)
    return
  end
  process_img(img, sz, t)
end


-- Listen at 100Hz
local cb_tbl = {
  houston = function() end
}
local fd_cam = assert(camera:descriptor())
local fd_updates = {
  [fd_cam] = update_read
}

local function entry()
  -- Set manual exposure
  camera:set_param("ae_mode", 1)
  -- Set exposure time in milliseconds
  camera:set_param("exposure_abs", 20.0)
  -- Begin to stream
  camera:stream_on()
end
entry()

racecar.listen{
  channel_callbacks = cb_tbl,
  fd_updates = fd_updates,
  -- loop_rate = 30,
  -- fn_loop = cb_loop
}
exit()
