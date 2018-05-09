package = "applanix"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-applanix.git"
}
description = {
  summary = "Read Applanix POS LV data stream",
  detailed = [[
      Read Applanix POS LV data stream
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-applanix",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["applanix"] = "applanix.lua",
  }
}
