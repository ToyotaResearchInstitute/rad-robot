package = "transform"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-transform.git"
}
description = {
  summary = "Manipulate transformation matrices",
  detailed = [[
    Performs mathematical operations of transformation matrices
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-transform",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["transform"] = "transform.lua",
  }
}
