package = "razor_imu"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-razor-imu.git"
}
description = {
  summary = "Razor IMU packets via Lua",
  detailed = [[
      Parse packets from Sparkfun Razor IMU M0 via Lua
    ]],
  homepage = "https://www.github.com/StephenMcGill-TRI/luajit-razor-imu",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["razor_imu"] = "razor_imu.lua",
  }
}
