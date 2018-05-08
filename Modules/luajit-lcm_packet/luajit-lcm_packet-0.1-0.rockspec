package = "luajit-lcm_packet"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-lcm.git"
}
description = {
  summary = "LCM packet generation",
  detailed = [[
    lcm-packet provides a layer for assembling and fragmenting packets using the LCM protocol in pure LuaJIT.
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-lcm",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",
  modules = {
    ["lcm_packet"] = "lcm_packet.lua",
  },
}
