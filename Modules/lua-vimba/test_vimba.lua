#!/usr/bin/env luajit

local vimba = require'vimba'

local ret = vimba.init()
print("init", ret)

local names = {"1930", "1920"}
local camera_ids = {"169.254.196.158", "169.254.104.127"}
local xml = {'GT1930C-settings.xml', "GT1920C-settings.xml"}

local cameras = {}
for i, CAM_ID in ipairs(camera_ids) do
  print("Opening", CAM_ID, names[i])
  local camera = vimba.open(CAM_ID)
  if camera then
    print("camera", camera)

    print("Loading XML:", xml[i])
    camera:load_xml(xml[i])

    print("Channel", names[i])
    camera:channel(names[i])

    print("Starting", names[i])
    camera:start()

    table.insert(cameras, camera)
  end
end

os.execute('sleep 10')

for i, camera in ipairs(cameras) do
  print("Stopping", camera)
  camera:stop()

  print("Closing", camera)
  camera:close()
end

print("Shutting down...")
vimba.shutdown()
