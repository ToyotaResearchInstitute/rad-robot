#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
local ch_pattern = arg[2] or ''

local logger = require'logger'
local sformat = require'string'.format
local tconcat = require'table'.concat
local tinsert = require'table'.insert

-- TODO: Use stringify
local has_cjson, cjson = pcall(require, 'cjson')
local stringify
if has_cjson then
  stringify = cjson.encode
else
  io.stderr:write"WARNING | No CJSON\n"
  stringify = function(obj, info)
    if type(info)~='table' then info = {} end
    for k, v in pairs(obj) do
      local tp = type(v)
      if tp=='table' and #v <= 12 then
        local vis = {}
        for ii,vv in ipairs(v) do
          vis[ii] = tonumber(vv) or ('"' .. tostring(vv) .. '"')
        end
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
    return "{"..tconcat(info, ',').."}"
  end
end

local function inspect(str, ch, t_us, count)
  if not ch:find(ch_pattern) then return end
  local obj = assert(logger.decode(str))
  if type(obj)=='string' then
    obj = {__payload = #obj}
  end
  -- Add extra bits
  obj['__channel'] = ch
  obj['__count'] = tonumber(count)
  obj['__t'] = tonumber(t_us)
  io.write(stringify(obj), '\n')
end

--
local options = {
  use_iterator = true,
}
local log = logger.open(fname)
local it_log, sz_log = log:play(options)
for str, ch, t_us, count in it_log do
  inspect(str, ch, t_us, count)
end
