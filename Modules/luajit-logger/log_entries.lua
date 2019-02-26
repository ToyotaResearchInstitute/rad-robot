#!/usr/bin/env luajit
local flags = require'racecar'.parse_arg(arg)
local logger = require'logger'
local sformat = require'string'.format
local unpack = unpack or require'table'.unpack

local lognames = {}
for _, fname in ipairs(flags) do
  assert(fname:match(logger.iso8601_match),
    string.format("Bad log datestamp: %s", fname))
  table.insert(lognames, fname)
end
assert(#lognames > 0, "No log files")

local include = flags.include or '.*'
local exclude = flags.exclude

local has_cjson, cjson = pcall(require, 'cjson')
local stringify
if has_cjson then
  stringify = cjson.encode
end

local function writeout(tbl, folder, t_us, cnt1)
  local tbl1 = {t_us = t_us}
  for ch1, tstr1 in pairs(tbl) do
    local _, str1 = unpack(tstr1)
    local obj1 = logger.decode(str1)
    if obj1.jpg then
      local basename = sformat("%s_%07d.jpg", ch1, cnt1)
      local outname = sformat("%s/%s", folder, basename)
      local f = assert(io.open(outname, "w"))
      f:write(obj1.jpg)
      f:close()
      obj1.jpg = basename
    elseif obj1.png then
      local basename = sformat("%s_%07d.png", ch1, cnt1)
      local outname = sformat("%s/%s", folder, basename)
      local f = assert(io.open(outname, "w"))
      f:write(obj1.png)
      f:close()
      obj1.png = basename
    end
    tbl1[ch1] = obj1
  end
  local str_json = stringify(tbl1)
  -- io.write(str_json, '\n')
  local basename = sformat("%07d.json", cnt1)
  local outname = sformat("%s/%s", folder, basename)
  local f = assert(io.open(outname, "w"))
  f:write(str_json)
  f:close()
end

local have_all_keys = false

local tbl = {}
for _, ch in logger.play(lognames, true) do
  local is_included = ch:find(include) ~= nil
  local is_excluded = exclude and (ch:find(exclude) ~= nil)
  if is_included and not is_excluded then
    tbl[ch] = false
  end
end

local function by_ts(t_usA, t_usB)
  return t_usA[3] < t_usB[3]
end

-- Sample at 10Hz
local folder = false
local log1 = false
local cnt1 = 0
local t_us_last = 0
local dt_us_sample = 0.1 * 1e6
for str, ch, t_us, cnt, ilog in logger.play(lognames, true) do
  local is_included = ch:find(include)
  local is_excluded = exclude and ch:find(exclude)
  if is_included and not is_excluded then
    -- Update the table
    tbl[ch] = {t_us, str}
    if not have_all_keys then
      have_all_keys = true
      for _, v in pairs(tbl) do
        if not v then have_all_keys = false; break end
      end
      if have_all_keys then
        -- Make the directory
        local _, datestamp = lognames[ilog]:match(logger.iso8601_match)
        folder = string.format("%s/harvest_%s", os.getenv"HOME", datestamp)
        io.stderr:write(sformat("Making directory [%s]\n", folder))
        assert(os.execute("mkdir -p "..folder))
        -- Save a smaller log in this directory
        if flags.save_log then
          log1 = assert(logger.new('harvest', folder, datestamp))
        end
      end
    end
    if have_all_keys and tonumber(t_us - t_us_last) >= dt_us_sample then
      -- Can write the lmp file of this subsampled log
      writeout(tbl, folder, tonumber(t_us), cnt1)
      cnt1 = cnt1 + 1
      if log1 then
        -- NOTE: This must be sorted by time!!
        local entries = {}
        for ch1, t_str in pairs(tbl) do
          local t_us1, str1 = unpack(t_str)
          table.insert(entries, {str1, ch1, t_us1})
        end
        table.sort(entries, by_ts)
        for _, entry in ipairs(entries) do
          assert(log1:write_raw(unpack(entry)))
        end
      end
      t_us_last = t_us
    end
  end
end
