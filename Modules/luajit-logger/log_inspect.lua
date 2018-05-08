#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
local ch_pattern = arg[2] or ''

local logger = require'logger'
local sformat = require'string'.format
local tconcat = require'table'.concat
local tinsert = require'table'.insert

assert(fname:match(logger.iso8601_match), "Bad log datestamp")

local function inspect(str, ch, t_us, count)
  if not ch:find(ch_pattern) then return end
  local obj = logger.decode(str)
  io.write(sformat("[%s][%d] # t_us: %d | %s\n",
    ch, tonumber(count), tonumber(t_us), type(obj)))
  if type(obj)=='string' then print(#str) end
  local info = {}
  for k, v in pairs(obj) do
    local tp = type(v)
    if tp=='table' and #v <= 12 then
      local vis = {}
      for ii,vv in ipairs(v) do vis[ii]=tostring(vv) end
      tinsert(info, sformat("%s: {%s}", k, tconcat(vis, ', ')))
      for kk, vv in pairs(v) do
        if type(kk)=='string' then
          tinsert(info, sformat("%s: {%s=%s}", k, kk, tostring(vv)))
        end
      end
    elseif tp=='string' and #v>128 then
      tinsert(info, sformat("%s: string[%d]", k, #v))
    else
      tinsert(info, sformat("%s: %s", k, tostring(v)))
    end
  end
  io.write(tconcat(info, '\n'), '\n\n')
end

for str, ch, t_us, count in logger.play(fname, true) do
  inspect(str, ch, t_us, count)
end
