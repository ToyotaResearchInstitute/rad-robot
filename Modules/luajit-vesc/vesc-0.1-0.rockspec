package = "vesc"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-vesc.git"
}
description = {
  summary = "VESC packets via LuaJIT FFI",
  detailed = [[
      Expose VESC interface via LuaJIT FFI
    ]],
  homepage = "https://www.github.com/StephenMcGill-TRI/luajit-vesc",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["vesc"] = "vesc.lua",
  }
}
