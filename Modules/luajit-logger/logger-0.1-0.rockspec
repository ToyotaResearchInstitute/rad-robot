package = "logger"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-logger.git"
}
description = {
  summary = "Log data in the LCM log format",
  detailed = [[
    Log using LCM log format
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-logger",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["logger"] = "logger.lua",
  }
}
