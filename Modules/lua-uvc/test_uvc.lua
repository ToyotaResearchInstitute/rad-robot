#!/usr/bin/env luajit
local uvc = require'uvc'

-- Serial: A94EF47E

local devname = arg[1]
--local cam = assert(uvc.init(devname, 1344, 376, 'yuyv', 1, 15))
local fmt = 'mjpeg'
-- local fmt = 'yuyv'
local fps = 5
local cam = assert(uvc.init(devname, 320, 240, fmt, 1, fps))
local fds = cam:descriptor()
print("FD", fds)
if type(fds)=='table' then
  print("FDs", unpack(fds))
end
-- local cam = assert(uvc.init(devname, 320, 240, fmt, 1, 15))
-- local cam = assert(uvc.init(devname, 320, 240, fmt, 1, 15))

print("ae_mode", cam:get_param("ae_mode"))
print("ae_priority", cam:get_param("ae_priority"))
print("exposure_abs", cam:get_param("exposure_abs"))
print("iris_abs", cam:get_param("iris_abs"))
print("white_balance_temperature_auto", cam:get_param("white_balance_temperature_auto"))
print("white_balance_temperature", cam:get_param("white_balance_temperature"))

-- Exposure priority
print("Set ae_mode", cam:set_param("ae_mode", 1))
print("Set exposure_abs", cam:set_param("exposure_abs", 20.0))
print("Set white_balance_temperature_auto", cam:set_param("white_balance_temperature_auto", 0))
print("Set white_balance_temperature", cam:set_param("white_balance_temperature", 3000))

print("ae_mode", cam:get_param("ae_mode"))
print("ae_priority", cam:get_param("ae_priority"))
print("exposure_abs", cam:get_param("exposure_abs"))
print("iris_abs", cam:get_param("iris_abs"))
print("white_balance_temperature_auto", cam:get_param("white_balance_temperature_auto"))
print("white_balance_temperature", cam:get_param("white_balance_temperature"))

cam:stream_on()

local t0 = os.time()
local tries = 0
local use_str = true
while tries < 2 * fps do
  -- os.execute("sleep "..tostring(timeout / 1e3))
  local img, size, count = cam:get_image(-1, use_str)
  if img==false then
    print("Error", size)
  else
    print(count, 'img', type(img), size)
  end
  tries = tries + 1
  if img then
    if fmt=='mjpeg' and type(img)=='string' then
      local fname_jpeg = "/tmp/uvc"..tostring(tries)..".jpeg"
      print("Writing to", fname_jpeg)
      local f_jpeg = assert(io.open(fname_jpeg, "w"))
      f_jpeg:write(img)
      f_jpeg:close()
    end
 else
   print("BAD IMAGE")
  end
end
local t1 = os.time()
print("Tries", tries, os.difftime(t1, t0), "seconds")

cam:close()
