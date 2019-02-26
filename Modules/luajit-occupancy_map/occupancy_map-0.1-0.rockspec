package = "occupancy_map"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-occupancy_map.git"
}
description = {
  summary = "SLAM with Lua",
  detailed = [[
  Occupancy map with scan matching support
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-occupancy_map",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",
  modules = {
    ["occupancy_map"] = "occupancy_map.lua",
  }
}
