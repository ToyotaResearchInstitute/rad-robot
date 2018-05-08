package = "velodyne_lidar"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-velodyne_lidar.git"
}
description = {
  summary = "Parse packets from a Velodyne LIDAR sensor",
  detailed = [[
      Parse Velodyne packets
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-velodyne_lidar",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["velodyne_lidar"] = "velodyne_lidar.lua",
  }
}
