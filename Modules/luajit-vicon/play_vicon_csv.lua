#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
assert(fname:match"%.csv$")
local f_csv = io.lines(fname)
local head_row = f_csv()

local keys = {}
for key in head_row:gmatch"[^,]+" do
  table.insert(keys, key)
end

for row in f_csv do
  local i = 0
  local info = {}
  for val in row:gmatch"[^,]+" do
    i = i + 1
    local key = keys[i]
    val = tonumber(val) or val
    info[key] = val
  end
  -- Process the information
end
