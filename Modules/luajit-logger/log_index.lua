#!/usr/bin/env luajit
--[[
fish $ for lmp in $HOME/harvest_ivs/*.lmp
  ./log_index.lua $lmp
end
]]
local logger = require'logger'
local fname = assert(arg[1], "No log specified")

local log = assert(logger.open(fname))
assert(log:index())
print(string.format("Entries in [%s]: [%d]", fname, tonumber(log.n_entries)))
