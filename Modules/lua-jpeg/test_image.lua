local jpeg = require 'jpeg'

local yuyv_filename = 'image.yuyv'
local yuyv_file = io.open(yuyv_filename, 'r')
local yuyv_str = yuyv_file:read('*all')

--local rgb_filename = 'image_rgb'
--local rgb_file = io.open(rgb_filename, 'r')
--local rgb_str = rgb_file:read('*a')

--for q=5,95,5 do
--	print("\nQuality",q)
--	jpeg.set_quality(q)
--	t0 = unix.time()
--	local yuyv_jpg = jpeg.compress_yuyv(yuyv_str, 640, 480)
--	t1 = unix.time()
--	local debug_yuyv = string.format("YUYV | Time: %.2f ms\tRatio: %2.2f%%\t%.2f kB",(t1-t0)*1000, 100*#yuyv_jpg/#yuyv_str, #yuyv_jpg/1024)
--
--	t0 = unix.time()
--	local rgb_jpg = jpeg.compress_rgb(rgb_str, 640, 480)
--	t1 = unix.time()
--	local debug_rgb = string.format("RGB  | Time: %.2f ms\tRatio: %2.2f%%\t%.2f kB",(t1-t0)*1000, 100*#rgb_jpg/#rgb_str, #rgb_jpg/1024)
--
--	print(debug_yuyv)
--	print(debug_rgb)
--end

print("YUYV")
do
  local c_yuyv = jpeg.compressor'yuyv'
  c_yuyv:downsampling(0)
  local yuyv_jpg
  for _=1,20 do
    yuyv_jpg = c_yuyv:compress(yuyv_str, 640, 480)
  end
  print("yuyv: ", #yuyv_jpg, 'bytes')
  local jpg_file = io.open('yuyv.jpg', 'w')
  jpg_file:write(yuyv_jpg)
  jpg_file:close()
  -- Downsample
  c_yuyv:downsampling(1)
  for _=1,20 do
    yuyv_jpg = c_yuyv:compress(yuyv_str, 640, 480)
  end
  print("yuyv downsampled: ", #yuyv_jpg, 'bytes')
  jpg_file = io.open('yuyv.downsampled.jpg', 'w')
  jpg_file:write(yuyv_jpg)
  jpg_file:close()
  -- Crop
  c_yuyv:downsampling(0)
  yuyv_jpg = c_yuyv:compress_crop(yuyv_str, 640, 480, 161, 121, 160, 120)
  jpg_file = io.open('yuyv.cropped.jpg', 'w')
  jpg_file:write(yuyv_jpg)
  jpg_file:close()
end

print("Y")
do
  local c_y = jpeg.compressor'y'
  c_y:downsampling(0)
  local y_jpg
  for _=1,20 do
    y_jpg = c_y:compress(yuyv_str, 640, 480)
  end
  print("y only:", #y_jpg, 'bytes')
  local jpg_file = io.open('y.jpg', 'w')
  jpg_file:write(y_jpg)
  jpg_file:close()
  -- Downsample
  c_y:downsampling(1)
  for _=1,20 do
    y_jpg = c_y:compress(yuyv_str, 640, 480)
  end
  print("y downsampled: ", #y_jpg, 'bytes')
  jpg_file = io.open('y.downsampled.jpg', 'w')
  jpg_file:write(y_jpg)
  jpg_file:close()
  -- Crop
  c_y:downsampling(0)
  y_jpg = c_y:compress_crop(yuyv_str, 640, 480, 161, 121, 160, 120)
  jpg_file = io.open('y.cropped.jpg', 'w')
  jpg_file:write(y_jpg)
  jpg_file:close()
end
