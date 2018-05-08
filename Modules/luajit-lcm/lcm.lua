local unpack = unpack or require'table'.unpack
local skt = require'skt'
local packet = require'lcm_packet'
local lib = {packet = packet}

-- Add a callback for a channel
local function register(self, channel, fn, decode)
  if type(channel)~='string' then
    return false, "Channel is not a string"
  elseif type(fn)~='function' then
    return false, "Callback is not a function"
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

local function update(self, timeout)
  local ret = skt.poll({self.skt_lcm.fd}, timeout)
  if ret == 0 then
    -- timeout reached
    return 0
  elseif ret < 0 then
    -- Some sort of socket error
    return false, "Bad socket"
  end
  local str, address, port = self.skt_lcm:recv(self)
  if str then
    local id = port and packet.gen_id(address, port)
    local channel, data = packet.assemble(str, #str, id)
    -- Run the callback
    if data then
      local decode = self.decoders[channel]
      local msg = decode and decode(data) or data
      local fn = self.callbacks[channel]
      if fn then fn(msg) end
    end
  end
  return ret
end

local function send(self, channel, msg)
  local enc = msg:encode()
  local frag = packet.fragment(channel, enc)
  return self.skt_lcm:send_all(frag)
end

function lib.init(_LCM_ADDRESS, _LCM_PORT, ch_cb)
  local LCM_ADDRESS = _LCM_ADDRESS or "239.255.76.67"
  local LCM_PORT = _LCM_PORT or 7667
  local skt_lcm, err = skt.open{
    address = LCM_ADDRESS,
    port = LCM_PORT,
  }
  if not skt_lcm then return false, err end
  local obj = {
    skt_lcm = skt_lcm,
    update = update,
    callbacks = {},
    decoders = {},
    register = register,
    send = send
  }
  if type(ch_cb)=='table' then
    for ch, fn in pairs(ch_cb) do obj:register(ch, fn) end
  end
  return obj
end

return lib
