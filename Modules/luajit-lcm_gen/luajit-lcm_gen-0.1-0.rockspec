package = "luajit-lcm_gen"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-lcm_gen.git"
}
description = {
  summary = "LCM data structures in LuaJIT",
  detailed = [[
    lcm-gen provides data structure generation for the LCM protocol in pure LuaJIT with no dependencies.
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-lcm_gen",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",
  install = {
    bin = {
      ["lcm_gen.lua"] = "lcm_gen.lua",
    }
  }
}
