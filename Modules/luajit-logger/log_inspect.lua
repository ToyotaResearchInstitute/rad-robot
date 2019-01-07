#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
local ch_pattern = arg[2] or ''

local logger = require'logger'
local sformat = require'string'.format
local tconcat = require'table'.concat
local tinsert = require'table'.insert

-- TODO: Use stringify
local has_cjson, cjson = pcall(require, 'cjson')
local stringify = false
if has_cjson then
  stringify = cjson.encode
end

assert(fname:match(logger.iso8601_match), "Bad log datestamp")

local function inspect(str, ch, t_us, count)
  if not ch:find(ch_pattern) then return end
  local obj = logger.decode(str)
  -- io.write(sformat("[%s][%d] # t_us: %d | %s\n",
  --   ch, tonumber(count), tonumber(t_us), type(obj)))
  local info = {
    string.format('"channel": "%s"', tostring(ch)),
    string.format('"count": %d', tonumber(count)),
    string.format('"t_us": %d', tonumber(t_us)),
  }
  if type(obj)=='string' then
    tinsert(info, sformat('"sz_payload": %d', #str))
    io.write("{\n", tconcat(info, ',\n'), '\n}\n')
    return
  end
  for k, v in pairs(obj) do
    local tp = type(v)
    if tp=='table' and #v <= 12 then
      local vis = {}
      for ii,vv in ipairs(v) do vis[ii] = tostring(vv) end
      if #vis>0 then
        tinsert(info, sformat('"%s": [%s]', k, tconcat(vis, ', ')))
      end
      for kk, vv in pairs(v) do
        if type(kk)=='string' then
          tinsert(info, sformat('"%s": {"%s": "%s"}', k, kk, tostring(vv)))
        end
      end
    elseif tp=='string' then
      if #v > 128 then
        tinsert(info, sformat('"%s": "uint8_t[%d]"', k, #v))
      else
        tinsert(info, sformat('"%s": "%s"', k, v))
      end
    else
      tinsert(info, sformat('"%s": %s', k, tostring(v)))
    end
  end
  io.write("{\n", tconcat(info, ',\n'), '\n}\n')
end

for str, ch, t_us, count in logger.play(fname, true) do
  inspect(str, ch, t_us, count)
end
