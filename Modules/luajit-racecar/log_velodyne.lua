#!/usr/bin/env luajit

local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local jitter_tbl = racecar.jitter_tbl
local log_announce = racecar.log_announce

local time = require'unix'.time
local poll = require'unix'.poll
local logger = require'logger'
local skt = require'skt'
local velodyne = require'velodyne_lidar'

local channels = {'velodyne', 'velodyne_nmea'}
local skts = {
  assert(skt.open{
    port = velodyne.DATA_PORT,
    use_connect = false
  }),
  assert(skt.open{
    port = velodyne.POSITION_PORT,
    use_connect = false
  })
}
local fds = {}
for i, s in ipairs(skts) do fds[i] = s.fd end
local process = {
  velodyne.parse_data, velodyne.parse_position
}

local log_dir = (os.getenv"RACECAR_HOME" or '.').."/logs"
local log = flags.log~=0 and assert(logger.new('velodyne', log_dir))

local function exit()
  if log then log:close() end
  for _, s in ipairs(skts) do s:close() end
end
-- racecar.handle_shutdown(exit)

local t_debug = time()
while racecar.running do
  local rc, ready = poll(fds)
  local t_poll = time()
  if rc and ready then
    for _, ind in ipairs(ready) do
      local pkt = skts[ind]:recv()
      local obj = process[ind](pkt)
      if obj then
        log_announce(log, obj, channels[ind])
      end
    end
  elseif rc then
    io.stderr:write("No data!\n")
  else
    io.stderr:write("uh oh\n")
  end
  local dt_debug = t_poll - t_debug
  if dt_debug > 1 then
    local info = jitter_tbl()
    io.write(table.concat(info, '\n'), '\n')
    t_debug = t_poll
  end
end
exit()
