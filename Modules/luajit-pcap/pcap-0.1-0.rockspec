package = "pcap"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/luajit-pcap.git"
}
description = {
  summary = "Iterate through a pcap file",
  detailed = [[
      Iterate through packets from pcap log files.
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/luajit-pcap",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "builtin",

  modules = {
    ["pcap"] = "pcap.lua",
  }
}
