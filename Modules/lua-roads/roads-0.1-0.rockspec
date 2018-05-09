package = "roads"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-roads.git"
}
description = {
  summary = "Parses an OSM file and extracts road information",
  detailed = [[
    Parses an OSM file and extracts road information and optionally output JSON
  ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-roads",
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
      'roads.lua',
    }
  }
}
