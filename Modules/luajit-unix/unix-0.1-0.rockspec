package = "unix"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-unix.git"
}
description = {
  summary = "Expose systems via LuaJIT FFI",
  detailed = [[
      Expose systems via LuaJIT FFI
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-unix",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["unix"] = "unix.lua",
  }
}
