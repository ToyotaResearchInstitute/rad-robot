#!/usr/bin/env luajit
local logger = require'logger'
local unpack = unpack or require'table'.unpack
local fname = arg[1]

local options = {
  use_iterator = true,
}

local log_copy = assert(logger.new('copy'))
print("Copy to", log_copy.log_name)
local log = assert(logger.open(fname))
local it_play = assert(log:play(options))
for str, ch, t_us, cnt in it_play do
  log_copy:write_raw(str, ch, t_us, cnt)
end
