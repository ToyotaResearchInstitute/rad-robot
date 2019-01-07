#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")

local logger = require'logger'

local has_cjson, cjson = pcall(require, 'cjson')
local stringify = has_cjson and cjson.encode
local _, datestamp = fname:match(logger.iso8601_match)
print(fname, fname:match(logger.iso8601_match))
assert(datestamp, "Bad log datestamp")
local folder = string.format("%s/%s", os.getenv"HOME", datestamp)
os.execute("mkdir -p "..folder)
local function inspect(str, ch, t_us, count)
  local obj = logger.decode(str)
  if obj.jpg then
    local outname = string.format("%s/%s_%06d.jpg", folder, ch, tonumber(count))
    local f = assert(io.open(outname, "w"))
    f:write(obj.jpg)
    f:close()
  elseif obj.png then
    local outname = string.format("%s/%s_%06d.png", folder, ch, tonumber(count))
    local f = assert(io.open(outname, "w"))
    f:write(obj.png)
    f:close()
  elseif stringify then
    local outname = string.format("%s/%s_%06d.json", folder, ch, tonumber(count))
    local f = assert(io.open(outname, "w"))
    f:write(stringify(obj))
    f:close()
  end
end

for str, ch, t_us, count in logger.play(fname, true) do
  inspect(str, ch, t_us, count)
end
