package = "slam"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-slam.git"
}
description = {
  summary = "SLAM with Lua",
  detailed = [[
  Simultaneous Localization and Mapping using scan matches from planar LIDAR
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-slam",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",
  modules = {
    ["slam"] = "slam.lua",
  }
}
