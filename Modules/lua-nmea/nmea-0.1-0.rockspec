package = "nmea"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-nmea.git"
}
description = {
  summary = "Parses NMEA sentences",
  detailed = [[
    Parses NMEA sentences
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-nmea",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = 'none',
  install = {
    bin = {
      'nmea.lua',
    }
  }
}
