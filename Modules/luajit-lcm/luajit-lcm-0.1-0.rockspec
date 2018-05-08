package = "luajit-lcm"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-lcm.git"
}
description = {
  summary = "LCM network processing",
  detailed = [[
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-lcm",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",
  modules = {
    ["lcm"] = "lcm.lua",
  },
}
