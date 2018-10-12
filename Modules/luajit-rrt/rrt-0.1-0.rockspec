package = "rrt"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-rrt.git"
}
description = {
  summary = "Rapidly-exploring Random Trees",
  detailed = [[
    Rapidly-exploring Random Trees with RRT* implementation:
    Sampling based algorihtms for optimal motion planning
    by Karaman and Frazzoli
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-rrt",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["rrt"] = "rrt.lua",
  }
}
