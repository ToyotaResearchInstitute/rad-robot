package = "hokuyo"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-hokuyo.git"
}
description = {
  summary = "Hokuyo packet processing",
  detailed = [[
      Expose Hokuyo interface via LuaJIT bitop
    ]],
  homepage = "https://www.github.com/StephenMcGill-TRI/luajit-hokuyo",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["hokuyo"] = "hokuyo.lua",
  }
}
