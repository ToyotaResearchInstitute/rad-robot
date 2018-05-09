package = "kalman"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-kalman.git"
}
description = {
  summary = "Kalman Filter",
  detailed = [[
    Linear Kalman filter
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-kalman",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["kalman"] = "kalman.lua",
  }
}
