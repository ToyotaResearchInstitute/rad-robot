package = "pcan"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-pcan.git"
}
description = {
  summary = "Access PCAN packets from the Ethernet-PCAN adapter",
  detailed = [[
      Decode CAN messages from ethernet frames captured with the PEAK CAN-Ethernet device
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-pcan",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["pcan"] = "pcan.lua",
  }
}
