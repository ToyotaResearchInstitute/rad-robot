#!/usr/bin/env luajit
local sformat = require'string'.format
local unpack = unpack or require'table'.unpack
local has_cjson, cjson = pcall(require, 'cjson')
local stringify = has_cjson and cjson.encode
local logger = require'logger'
-- Parse the flags
local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
-- Grab the log names
local lognames = {unpack(flags)}
table.sort(lognames)
io.stderr:write(sformat("Processing [%s]\n", table.concat(lognames, ', ')))
-- Find the filehash
local filehash = io.popen(string.format("echo '%s' | shasum", lognames[1])):read"*line"
filehash = filehash:match"^[a-z,0-9]+"
io.stderr:write(sformat("Filehash [%s]\n", filehash))
if flags.filehash then
  -- Simply print the hash
  os.exit()
end

-- Make the directory
local folder = string.format("%s/harvests/%s", racecar.ROBOT_HOME, filehash)
io.stderr:write(sformat("Making directory [%s]\n", folder))
assert(os.execute("mkdir -p "..folder))
-- Place one folder above
local log_harvest = assert(logger.new("harvest", string.format("%s/harvests", racecar.ROBOT_HOME), filehash))

local count = 0
local function harvest(messages, t_us)
  -- Write the LMP harvest file
  log_harvest:write(messages, nil, t_us)
  -- Form a table that has all the message information
  local tbl = {}
  count = count + 1
  for ch, obj in pairs(messages) do
    -- Write out the images
    for _, ext in ipairs{'jpg', 'png'} do
      local str_im = obj[ext]
      if str_im then
        local basename = sformat("%s_%07d.%s", ch, count, ext)
        local outname = sformat("%s/%s", folder, basename)
        local f_img = assert(io.open(outname, "w"))
        f_img:write(str_im)
        f_img:close()
        obj[ext] = basename
      end
    end
    -- Concatenate all keys
    tbl[ch] = obj
  end
  local str_json = stringify(tbl)
  local basename = sformat("%07d.json", count)
  local outname = sformat("%s/%s", folder, basename)
  local f_json = assert(io.open(outname, "w"))
  f_json:write(str_json)
  f_json:close()
end

-- Use the object format - with non-unique data, we retain slow messages
-- NOTE: This is stale, but can be useful at higher message rates
-- NOTE: In obj_fmt, be careful about high sample rates
local options = {
  use_obj_fmt = true,
  -- dt_sample = 0.1, -- 10Hz sampling
  dt_sample = 'video0', -- 10Hz sampling
  include = flags.include,
  exclude = flags.exclude,
}
assert(logger.sync(harvest, lognames, options))
-- Index the harvested LMP file
-- io.stderr:write(string.format("Indexing [%s]\n", log_harvest.log_name))
-- assert(log_harvest:close():open():index())
