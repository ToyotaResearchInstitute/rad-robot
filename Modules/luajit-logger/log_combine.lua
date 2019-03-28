#!/usr/bin/env luajit
local logger = require'logger'
local unpack = unpack or require'table'.unpack
local outname = arg[1]
local lognames = {unpack(arg, 2)}

assert(logger.new(outname)):combine(lognames)
