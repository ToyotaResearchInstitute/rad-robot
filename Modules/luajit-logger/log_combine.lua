#!/usr/bin/env luajit
local logger = require'logger'
local outname = arg[1]

local lognames = {}
for i=2,#arg do
  local fname = assert(arg[i], "No log specified")
  assert(fname:match(logger.iso8601_match), "Bad log datestamp")
  table.insert(lognames, fname)
end
assert(#lognames>1, "Combine two or more log files")

return assert(logger.new(outname)):combine(lognames):close()
