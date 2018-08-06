#!/usr/bin/env luajit
local fname = assert(arg[1], "No log specified")
local ch_pattern = arg[2] or ''

local logger = require'logger'
local sformat = require'string'.format
local tconcat = require'table'.concat
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack

assert(fname:match(logger.iso8601_match), "Bad log datestamp")

local function vicon2pose(vp)
  return vp.translation[1] / 1e3, vp.translation[2] / 1e3, vp.rotation[3]
end

local fp = io.stdout

fp:write("time,frame,clock,vehicle,x,y,yaw\n")
local frame0, t_us0
local entries = {}

local n_cars = 7
local frames_per_second = 120

local function pop_and_write()
  local entry = table.remove(entries)
  if not entry then return 0 end
  if not frame0 then frame0 = entry.frame end
  local dframe = entry.frame - frame0
  local dt = dframe / frames_per_second
  fp:write(sformat("%f,%d,%f,%s,%f,%f,%f\n",
                   dt, entry.frame, entry.dt, entry.name, unpack(entry.pose)))
  return #entries
end

local function inspect(str, ch, t_us, count)
  if not ch:find(ch_pattern) then return end
  local obj = logger.decode(str)
  if type(obj)~='table' then return false, "bad data" end
  local frame = obj.frame
  if not frame then return false, "Not vicon data" end
  if not t_us0 then t_us0 = t_us end
  local dt_us = t_us - t_us0
  io.stderr:write(sformat("Frame %d\n", frame))

  for name, info in pairs(obj) do
    if type(info)=='table' and info.translation then
      local dt = frame / frames_per_second
      local entry = {
                     frame = frame,
                     pose = {vicon2pose(info)},
                     dt = tonumber(dt_us) / 1e6,
                     name = name
                    }
      for i=1,#entries do
        local prev_entry = entries[i]
        if frame > prev_entry.frame then
          table.insert(entries, i, entry)
          break
        end
      end
      if #entries == 0 then
        table.insert(entries, entry)
      end
    end
  end

  if #entries >= 13 * n_cars then pop_and_write() end
  return frame
end

for str, ch, t_us, count in logger.play(fname, true) do
  assert(inspect(str, ch, t_us, count))
end

repeat local n = pop_and_write() until n <= 0