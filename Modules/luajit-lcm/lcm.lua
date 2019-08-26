local lib = {}
local nan = 0/0
local min = require'math'.min
local max = require'math'.max
local sformat = require'string'.format
local tinsert = require'table'.insert
local tremove = require'table'.remove
local unpack = unpack or require'table'.unpack

local skt = require'skt'
local has_unix, unix = pcall(require, 'unix')
local time_us = has_unix and unix.time_us
local lcm_packet = require'lcm_packet'

-- Calculate the jitter in milliseconds
local function get_jitter(times_us)
  if #times_us<2 then return nan, nan, nan end
  local diffs, adiff = {}, 0
  for i=2,#times_us do
    local d = tonumber(times_us[i] - times_us[i-1])
    adiff = adiff + d
    tinsert(diffs, d)
  end
  adiff = adiff / #diffs
  local jMin, jMax = min(unpack(diffs)), max(unpack(diffs))
  -- milliseconds: Average, minimum and maximum
  return adiff/1e3, (jMin - adiff)/1e3, (jMax - adiff)/1e3
end
local function jitter_info(self, use_send)
  local info = {}
  local jitter_times = use_send and self.jitter_times_send or self.jitter_times_recv
  for ch, ts in pairs(jitter_times) do
    local avg, jitterA, jitterB = get_jitter(ts)
    tinsert(info, sformat(
      "%s\t%5.1f Hz\t%+6.2f ms\t%6.2f ms\t%+6.2f ms",
      ch, 1e3/avg, jitterA, avg, jitterB))
  end
  return info
end

-- Add a callback for a channel
local function lcm_register(self, channel, fn, decode)
  if type(channel)~='string' then
    return false, "Channel is not a string"
  elseif type(fn)~='function' then
    return false, "Callback is not a function"
  elseif type(decode)~='function' then
    return false, "Decoder is not a function"
  end
  self.callbacks[channel] = fn
  self.decoders[channel] = decode
  return self
end

-- Update this LCM channel
-- NOTE: Should poll before this
local function lcm_receive0(self)
  local pkts, err = self.skt:recvmmsg(false)
  if not pkts then return false, err end
  for _, pkt in ipairs(pkts) do
    local str, address, port, ts = unpack(pkt)
    -- Grab the time in microseconds
    local t_us = time_us(ts)
    local id = port and lcm_packet.gen_id(address, port)
    local channel, data = self.partitioner:assemble(str, #str, id)
    -- Run the callback
    if channel and data then
      local count = (self.count_recv[channel] or 0) + 1
      self.count_recv[channel] = count
      local fn = self.callbacks[channel]
      if fn then
        local decode = self.decoders[channel]
        local msg = decode and decode(data) or data
        fn(msg, channel, t_us, count)
      end
      -- Update the jitter information
      local jitter_times = self.jitter_times_recv[channel]
      if not jitter_times then
        self.jitter_times_recv[channel] = {t_us}
      else
        tinsert(jitter_times, t_us)
        if #jitter_times > self.jitter_window then
          tremove(jitter_times, 1)
        end
      end
    end -- if ch and data
  end -- loop each message
end

-- Should poll before this
local function lcm_receive(self)
  -- io.stderr:write("\n== lcm_receive start ==\n")
  -- local str, address, port, ts = self.skt:recvmsg()
  -- print(#str, "str", str)
  local str, address, port, ts = self.skt:recv()
  -- io.stderr:write("ts: ", tostring(ts), "\n")
  -- Grab the time in microseconds
  local t_us = time_us(ts)
  -- io.stderr:write("t_us: ", tostring(t_us), "\n")
  if type(str)~='string' then return false, "No data" end
  -- io.stderr:write("port: ", port, "\n")
  -- io.stderr:write("address: ", address, "\n")
  local id = port and lcm_packet.gen_id(address, port)
  -- io.stderr:write("id: ", tostring(id), "\n")
  local channel, data = self.partitioner:assemble(str, #str, id)
  -- io.stderr:write("assemble: ", tostring(channel), "\n")
  -- if not channel then
  --   io.stderr:write("assemble err: ", tostring(data), "\n")
  -- end
  if type(channel)~='string' then return false, "Bad assemble" end
  -- If not a string, then there is no full message
  if type(data)~='string' then return end
  -- Update the message count
  local count = (self.count_recv[channel] or 0) + 1
  -- io.stderr:write("count: ", tostring(count), "\n")
  self.count_recv[channel] = count
  -- Run the callback, if it exists
  local fn = self.callbacks[channel]
  if fn then
    local decode = self.decoders[channel]
    local msg = decode and decode(data) or data
    fn(msg, channel, t_us, count)
  end
  -- Update the jitter information
  local jitter_times = self.jitter_times_recv[channel]
  if not jitter_times then
    self.jitter_times_recv[channel] = {t_us}
  else
    tinsert(jitter_times, t_us)
    if #jitter_times > self.jitter_window then
      tremove(jitter_times, 1)
    end
  end
  -- io.stderr:write("\n== lcm_receive finish ==\n")
end

local function lcm_send(self, channel, msg)
  -- io.stderr:write("\n== lcm_send start ==\n")
  -- Allow a custom encoding
  local enc = type(msg)=='string' and msg or msg:encode()
  -- The count is required for encoding
  local count = (self.count_send[channel] or 0) + 1
  -- Fragment, now, and grab the count
  -- io.stderr:write("Fragment\n")
  local frag = self.partitioner:fragment(channel, enc, #enc, count)
  -- Return the number of bytes sent
  -- io.stderr:write("Send\n")
  local n_bytes_sent, err = self.skt:send_all(frag)
  -- io.stderr:write("Sent", tostring(n_bytes_sent),"\n")
  local t_us = time_us()
  -- Update the jitter information
  local jitter_times = self.jitter_times_send[channel]
  if not jitter_times then
    self.jitter_times_send[channel] = {t_us}
  else
    tinsert(jitter_times, t_us)
    if #jitter_times > self.jitter_window then
      tremove(jitter_times, 1)
    end
  end
  -- Update the count
  self.count_send[channel] = count
  -- io.stderr:write("\n== lcm_send finish ==\n")
  return n_bytes_sent, err
end

-- TODO: Add other file descriptors to listen for
local function lcm_update(self, timeout)
  -- print("Listening on", unpack(self.fds))
  local ret, events = skt.poll(self.fds, timeout)
  if ret == 0 then
    -- timeout reached
    return 0
  elseif not ret then
    -- Some sort of socket error
    print"error"
    return false, events
  end
  for i, e in ipairs(events) do
    -- print("Update", i, e)
    if e then self.fd_updates[i](e) end
  end
  return ret
end

local function fd_register(self, fd, update)
  if type(fd)~='number' then
    return false, "File descriptor is not a number"
  elseif type(update)~='function' then
    return false, "Update is not a function"
  end
  table.insert(self.fds, fd)
  table.insert(self.fd_updates, update)
  return self
end

function lib.init(options)
  options = type(options)=='table' and options or {}
  local skt_lcm = options.skt
  if type(skt_lcm)~='table' then
    local LCM_ADDRESS = options._LCM_ADDRESS or "239.255.76.67"
    local LCM_PORT = options._LCM_PORT or 7667
    local err
    skt_lcm, err = skt.open{
      address = LCM_ADDRESS,
      port = LCM_PORT,
      ttl = tonumber(options.ttl) or 0
    }
    if not skt_lcm then return false, err end
  end
  local obj = {
    skt = skt_lcm,
    callbacks = {},
    decoders = {},
    count_send = {},
    count_recv = {},
    -- Jitter information
    jitter_times_send = {},
    jitter_times_recv = {},
    jitter_window = 100,
    jitter_info = jitter_info,
    -- Exposed functions
    cb_register = lcm_register,
    send = lcm_send,
    receive = lcm_receive,
    update = lcm_update,
    --
    partitioner = lcm_packet.new_partitioning(options.mtu)
  }
  -- File descriptors and their on-data updates
  obj.fds = {obj.skt.fd}
  obj.fd_updates = {function() lcm_receive(obj) end}
  obj.fd_register = fd_register
  return obj
end

return lib
