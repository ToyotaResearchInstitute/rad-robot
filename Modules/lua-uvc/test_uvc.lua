#!/usr/bin/env luajit
local uvc = require'uvc'

local devname = arg[1] or '/dev/video0'
--local cam = assert(uvc.init(devname, 1344, 376, 'yuyv', 1, 15))
local cam = assert(uvc.init(devname, 320, 240, 'mjpeg', 1, 15))
local timeout = tonumber(arg[2]) or -1

local t0 = os.time()
local has_img = false
local tries = 0
while not has_image do
  
  local img, size = cam:get_image(timeout)
  tries = tries + 1
  if img then
    print('img', img, size)
    has_image = true
--  else
--    print("BAD IMAGE")
  end
end
local t1 = os.time()
print(tries, os.difftime(t1, t0))

cam:close()
