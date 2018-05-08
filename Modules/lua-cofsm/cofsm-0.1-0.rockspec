package = "cofsm"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGIll-TRI/lua-cofsm.git"
}
description = {
  summary = "Coroutine Finite State Machine",
  detailed = [[
    Run a finite state machine with coroutines attached to states
  ]],
  homepage = "https://github.com/StephenMcGIll-TRI/lua-cofsm",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["cofsm"] = "cofsm.lua",
  }
}
