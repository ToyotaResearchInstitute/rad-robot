package = "quaternion"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-quaternion.git"
}
description = {
  summary = "Manipulate quaternions",
  detailed = [[
    Performs mathematical operations of quaternions
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-quaternion",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["quaternion"] = "quaternion.lua",
  }
}
