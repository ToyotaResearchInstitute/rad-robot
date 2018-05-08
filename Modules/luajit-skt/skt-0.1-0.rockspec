package = "skt"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-skt.git"
}
description = {
  summary = "Datagram sending and receiving for UNIX, including multicast",
  detailed = [[
      Provides access to TCP, UDP, UNIX sockets.
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-skt",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["skt"] = "skt.lua",
  }
}
