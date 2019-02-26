#!/usr/bin/env luajit
-- L:S1117709, C:S1111161, R:S1117069
print("Loading the library")
local dalsa = require'dalsa'

local interface_addresses = {}
local ifconfig = io.popen"ifconfig":read"*all"
for addr in ifconfig:gmatch"inet addr:(%d+%.%d+%.%d+%.%d+)" do
--  if addr:match"^169%.254%." then
    table.insert(interface_addresses, addr)
--  end
end

dalsa.initialize(interface_addresses)

local cameras = dalsa.list()
if not cameras then
  return
end
print("Serial numbers", unpack(cameras))

print("\nFinding camera")
local camera, serial = dalsa.open(arg[1])
print("Open", camera, serial)

if not camera then
  local ret, err = dalsa.shutdown()
  print("Shutdown", ret, err)
  os.exit()
end

local keys = {
  "PixelFormat", "Width", "Height", "PayloadSize",
  "BalanceWhiteAuto", "ChunkModeActive","autoBrightnessMode",
  "DeviceSerialNumber", 
}
local parameters = {}
for _, key in ipairs(keys) do
  local val, err = camera:get_parameter(key)
  print(key, val, err)
  parameters[key] = val
end

print("Set wb", camera:set_parameter("BalanceWhiteAuto", "Periodic"))
print("Set ab", camera:set_parameter("autoBrightnessMode", "Active"))
print("Set metadata", camera:set_parameter("ChunkModeActive", 0))

print("\nGet XML")
local name, xml = camera:get_xml()
print(type(xml), name)
if type(name)=='string' then
  name = name:match("([^/]+%.xml)$")
  print("Name", name)
end
if name and type(xml)=='string' then
  print("Saving XML")
  local f = io.open(name, 'w')
  f:write(xml)
  f:close()
end

local frame_ptr, frame_str
if camera then
  print("\nStream on")
  local ret, err = camera:stream_on(string.format("camera_%s", serial))

  for i=1,5 do
    print("\nGet Frame", i)
    frame_ptr, frame_str = camera:get_image()
    if not frame_ptr then
      print(frame_str)
    else
      print("Frame", frame_ptr, #frame_str)
    end
  end

  print("\nStream off")
  local ret, err = camera:stream_off()

  print("\nClosing")
  local ret, err = camera:close()
  print("Close", ret, err)
end

os.execute'sleep 1'
local ret, err = dalsa.shutdown()
print("\nShutdown", ret, err)

if frame_ptr and type(frame_str)=='string' then
  print("Saving raw image")
  local f = io.open("image.raw", 'w')
  f:write(frame_str)
  f:close()
  local has_jpeg, jpeg = pcall(require, 'jpeg')
  if has_jpeg then
    print("Saving JPEG image")
    local width = assert(tonumber(parameters.Width)) / 2
    local height = assert(tonumber(parameters.Height)) / 2
    local c_rgb = jpeg.compressor'rgb'
    local frame_jpg = c_rgb:compress(frame_str, width, height)
    if type(frame_jpg)=='string' then
      print(#frame_jpg)
      local f = io.open("image.jpeg", 'w')
      f:write(frame_jpg)
      f:close()
    end
  end
end

