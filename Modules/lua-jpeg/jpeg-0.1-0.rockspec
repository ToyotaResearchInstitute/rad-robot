package = "jpeg"
version = "0.1-0"
source = {
  url = "git://github.com/StephenMcGill-TRI/lua-jpeg.git"
}
description = {
  summary = "JPEG Compression library",
  detailed = [[
      Compress/decompress JPEG images
    ]],
  homepage = "https://github.com/StephenMcGill-TRI/lua-jpeg",
  maintainer = "Stephen McGill <stephen.mcgill@tri.global>",
  license = "MIT"
}
dependencies = {
  "lua >= 5.1",
}
build = {
  type = "make",
  build_variables = {
    CFLAGS="$(CFLAGS)",
    LIBFLAG="$(LIBFLAG)",
    LUA_LIBDIR="$(LUA_LIBDIR)",
    LUA_BINDIR="$(LUA_BINDIR)",
    LUA_INCDIR="$(LUA_INCDIR)",
    LUA="$(LUA)",
  },
  install_variables = {
    INST_PREFIX="$(PREFIX)",
    INST_BINDIR="$(BINDIR)",
    INST_LIBDIR="$(LIBDIR)",
    INST_LUADIR="$(LUADIR)",
    INST_CONFDIR="$(CONFDIR)",
  },
}
