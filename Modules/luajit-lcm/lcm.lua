local unpack = unpack or require'table'.unpack
local skt = require'skt'
local lcm_packet = require'lcm_packet'
local lib = {}

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
--[[
local function update0(self, timeout)
  if timeout~=0 then skt.poll({self.skt_lcm.fd}, timeout) end
  local pkts, err = self.skt_lcm:recvmmsg(false)
  if not pkts then return false, err end
  for _, pkt in ipairs(pkts) do
    local str, address, port = unpack(pkt)
    local id = port and packet.gen_id(address, port)
    local channel, data = packet.assemble(str, #str, id)
    -- Run the callback
    if data then
      local decode = self.decoders[channel]
      local msg = decode and decode(data) or data
      local fn = self.callbacks[channel]
      local ret = fn and fn(msg)
    end
  end
end
--]]

-- Should poll before this
local function lcm_receive(self, timeout)
  local str, address, port = self.skt:recv(timeout)
  if type(str)~='string' then
    return false, "No data"
  end
  local id = port and lcm_packet.gen_id(address, port)
  local channel, data = self.partitioner:assemble(str, #str, id)
  if not channel then return false, "Bad assemble" end
  -- Run the callback
  local fn = self.callbacks[channel]
  if fn and type(data)=='string' then
    local decode = self.decoders[channel]
    local msg = decode and decode(data) or data
    fn(msg)
  end
end

local function lcm_send(self, channel, msg)
  local enc = msg:encode()
  local frag = self.partitioner:fragment(channel, enc)
  return self.skt:send_all(frag)
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
