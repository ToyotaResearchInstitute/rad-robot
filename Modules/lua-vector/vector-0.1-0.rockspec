package = "vector"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-vector.git"
}
description = {
  summary = "Manipulate vectors",
  detailed = [[
    Performs operations on table of numbers
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-vector",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["vector"] = "vector.lua",
  }
}
