local bor = require'bit'.bor
local lshift = require'bit'.lshift
local ffi = require'ffi'
local C = ffi.C

local ceil = require'math'.ceil
local min = require'math'.min
local tinsert = require'table'.insert

local lib = {}

-- Packet reassembly
local MAGIC_LCM2 = 0x4c433032
local MAGIC_LCM3 = 0x4c433033

local LCM3_NUM_BUFFERS = 4
local LCM3_MAX_FRAGMENTS = 256
-- local LCM_MAX_CHANNEL_LENGTH = 256
local LCM_MAX_CHANNEL_LENGTH = 63

-- local MAXIMUM_HEADER_LENGTH = 300

local IP_HEADER_SZ = 20 -- Can go up to 60
local UDP_HEADER_SZ = 8
-- Nested with the UDP header are _more_ headers
local LCM2_HEADER_SZ = 8
local LCM3_HEADER_SZ = 20

local partition_sizes = setmetatable({
  ['OSX'] = 1443, -- Not sure why smaller... 1435 in lcm udpm
  ['wifi'] = 1500 - IP_HEADER_SZ - UDP_HEADER_SZ,
  -- ['lite'] = 8192, -- In lcm-lite
  ['localhost'] = 2^14 - IP_HEADER_SZ - UDP_HEADER_SZ,
  -- (2^16 - 1) is 0xFFFF
  ['jumbo'] = (2^16 - 1) - IP_HEADER_SZ - UDP_HEADER_SZ,
}, {
__call = function(self, mtu)
  if type(mtu) == 'number' then
    return mtu - IP_HEADER_SZ - UDP_HEADER_SZ
  elseif type(mtu) == 'string' then
    return self[mtu]
  end
  return self.wifi
end
})

ffi.cdef[[
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);
uint32_t htonl(uint32_t hostlong);
uint32_t htons(uint16_t hostlong);
]]

local function encode_u32(buf, num)
  -- Try to just use the built-in...
  -- NOTE: this may not work on embedded systems, however
  ffi.cast("uint32_t*", buf)[0] = C.htonl(num)
end

local function decode_u32(buf)
  -- Try to just use the built-in...
  -- NOTE: this may not work on embedded systems, however
  return C.ntohl(ffi.cast('uint32_t*', buf)[0])
end

local function encode_u16(buf, num)
  -- Try to just use the built-in...
  -- NOTE: this may not work on embedded systems, however
  ffi.cast("uint16_t*", buf)[0] = C.htons(num)
end

local function decode_u16(buf)
  -- Try to just use the built-in...
  -- NOTE: this may not work on embedded systems, however
  return C.ntohs(ffi.cast('uint16_t*', buf)[0])
end

ffi.cdef[[
size_t strnlen(const char *s, size_t maxlen);
]]
local function assemble2(self, buf, buf_len, msg_id, msg_seq)
  local buf_pos = 8
  -- copy zero-terminated string holding the channel #
  local cap_sz = min(buf_len - buf_pos, self.LCM_MAX_CHANNEL_LENGTH)
  local ch_sz = tonumber(C.strnlen(buf + buf_pos, cap_sz))
  if ch_sz == cap_sz then
    return false, string.format("Bad name: %d / %d", ch_sz, cap_sz)
  end
  local channel = ffi.string(buf + buf_pos)
  buf_pos = buf_pos + ch_sz + 1 -- Plus the null terminator
  -- Return the Channel and Payload
  local payload_sz = buf_len - buf_pos
  if payload_sz < 0 then return false, "Bad payload" end
  local payload = ffi.string(buf + buf_pos, payload_sz)
  return channel, payload
end

local function get_buffer(fragment_buffers, msg_id, msg_seq, msg_size, fragments_in_msg)
  -- Search for fragment
  for i, f in ipairs(fragment_buffers) do
    if f.msg_id == msg_id and f.msg_seq == msg_seq and f.msg_size == msg_size and f.fragments_in_msg == fragments_in_msg then
      return f, i
    end
  end
  local fbuf, ibuf
  if #fragment_buffers >= LCM3_NUM_BUFFERS then
    -- Find the oldest packet
    local old_seq = math.huge
    for i, f in ipairs(fragment_buffers) do
      -- Use a completed packet, while waiting on the others
      if f.fragments_remaining==0 then
        ibuf = i
        fbuf = f
        break
      elseif f.msg_seq < old_seq then
        old_seq = f.msg_seq
        ibuf = i
        fbuf = f
      end
    end
  else
    fbuf = {}
    tinsert(fragment_buffers, fbuf)
    ibuf = #fragment_buffers
  end

  -- Initialize the buffer
  fbuf.msg_id = msg_id --sender id
  fbuf.msg_seq = msg_seq
  fbuf.msg_size = msg_size
  fbuf.buf = ffi.new('uint8_t[?]', msg_size)
  fbuf.fragments_in_msg = fragments_in_msg
  fbuf.frag_received = {} -- just accumulate the received packets
  fbuf.fragments_remaining = fragments_in_msg

  return fbuf, ibuf
end

local function assemble3(self, buf, buf_len, msg_id, msg_seq)
  local buf_pos = 8 -- Have started already
  local msg_size = decode_u32(buf + buf_pos)
  buf_pos = buf_pos + 4
  local fragment_offset = decode_u32(buf + buf_pos)
  buf_pos = buf_pos + 4
  local fragment_id = decode_u16(buf + buf_pos)
  buf_pos = buf_pos + 2
  local fragments_in_msg = decode_u16(buf + buf_pos)
  buf_pos = buf_pos + 2

  -- Add sanity checks
  local payload_len = buf_len - buf_pos
  if payload_len < 1 then
    return false, "Bad payload size"
  end

  if fragments_in_msg > LCM3_MAX_FRAGMENTS then
    return false, string.format("%d > LCM3_MAX_FRAGMENTS", fragments_in_msg)
  end

  if fragment_id >= fragments_in_msg then
    return false, string.format("Invalid fragment ID %d/%d",
                                fragment_id, fragments_in_msg)
  end

  if fragment_offset + payload_len > msg_size then
    return false, string.format(
      "Invalid fragment size %d + %d = %d > %d",
      fragment_offset, tonumber(payload_len), tonumber(fragment_offset + payload_len), msg_size)
  end

  -- Find our buffer
  local fbuf = get_buffer(self, msg_id, msg_seq, msg_size, fragments_in_msg)
  -- Check if the buffer has been completed already
  if fbuf.fragments_remaining == 0 then
    return false, "Redudant packet"
  elseif fbuf.frag_received[fragment_id + 1] then
    return false, "Redundant fragment"
  end

  -- First fragment contains the channel name plus data
  if fragment_id == 0 then
    local cap_sz = min(buf_len - buf_pos, self.LCM_MAX_CHANNEL_LENGTH)
    local ch_sz = tonumber(C.strnlen(buf + buf_pos, cap_sz))
    if ch_sz == cap_sz then
      return false, string.format("Bad name: %d / %d", ch_sz, cap_sz)
    end
    fbuf.channel = ffi.string(buf + buf_pos)
    buf_pos = buf_pos + ch_sz + 1
  end

  -- Copy the payload fragment into the buffer
  local payload_sz = buf_len - buf_pos
  if payload_sz > 0 then
    ffi.copy(fbuf.buf + fragment_offset, buf + buf_pos, payload_sz)
  end

  -- Set as received
  fbuf.frag_received[fragment_id+1] = true
  -- Decerement number of fragments we need for a full packet
  fbuf.fragments_remaining = fbuf.fragments_remaining - 1
  -- Debug
  --[[
  local pkts={}
  for i=1,fragments_in_msg do
    pkts[i] = fbuf.frag_received[i] and 'X' or '-'
  end
  print("Packets:", table.concat(pkts))
  --]]

  -- Check if we are done assembling this fragment
  if fbuf.fragments_remaining == 0 then
    -- Retain in the buffer, in case redundant packets were sent for reliability
    -- When receiving a redundant packet for a message that already was assembled,
    -- the buffer is aware
    return fbuf.channel, ffi.string(fbuf.buf, msg_size)
  else
    -- Still need packets for assembling
    return fbuf.channel or true
  end

end

-------------------
-- Assembly area --
-------------------
-- Buffer is a uint8_t*
local function assemble(self, buffer, buf_len, msg_id)
  if not buffer then return false, "Bad input" end
  -- if buf_len < 4 then return false, "Header too small" end
  if buf_len < 8 then return false, "Header too small" end
  local buf = ffi.cast('uint8_t*', buffer)
  local magic = decode_u32(buf)
  local msg_seq = decode_u32(buf + 4)
  if magic==MAGIC_LCM2 then
    return assemble2(self, buf, buf_len, msg_id or 0, msg_seq)
  elseif magic==MAGIC_LCM3 then
    -- local nfrags = decode_u16(buf + 18)
    -- print("nfrags", nfrags)
    return assemble3(self, buf, buf_len, msg_id or 0, msg_seq)
  else
    return false, "Bad magic number"
  end
end

------------------------
-- Fragmentation area --
------------------------

-- Channel is a string
-- TODO: Message is string or void*
-- TODO: Have a pre-existing buffer for message sending...
-- Smaller buffer creation
local function frag2(self, channel, ch_sz, message, msg_sz, msg_seq)
  if type(msg_seq)~='number' then
    return false, "Need a message sequence number"
  end
  -- Assemble non-fragmented message
  local buf_pos = 0
  local buf = ffi.new('uint8_t[?]', self.LCM_MAX_MESSAGE_SZ)
  local msg = ffi.cast("uint8_t*", message)
  -- Set the header identifier
  encode_u32(buf + buf_pos, MAGIC_LCM2)
  buf_pos = buf_pos + 4
  -- TODO: Track the message sequence
  encode_u32(buf + buf_pos, msg_seq)
  buf_pos = buf_pos + 4
  -- copy channel (and null terminator)
  ffi.copy(buf + buf_pos, channel)
  buf_pos = buf_pos + ch_sz + 1

  ffi.copy(buf + buf_pos, msg, msg_sz)
  buf_pos = buf_pos + msg_sz

  -- Give a Lua string
  -- TODO: Choose between Lua string and ffi array
  return ffi.string(buf, buf_pos)
end

-- TODO: Make this coroutine based
local function frag3(self, channel, ch_sz, message, msg_len, msg_seq)
  if type(msg_seq)~='number' then
    return false, "Need a message sequence number"
  end
  -- TODO: Maximum channel size
  -- TODO: Is this wrong?
  local fragments_in_msg = ceil((msg_len + ch_sz) / self.LCM3_MAX_FRAGMENT_SZ)
  if fragments_in_msg > self.MAX_NUM_FRAGMENTS then
    return false, "Message requires too many fragments"
  end

  --print("MAX FRAG, MSG", self.LCM3_MAX_FRAGMENT_SZ, self.LCM_MAX_MESSAGE_SZ)

  local msg = ffi.cast("uint8_t*", message)
  -- Table-based for now...
  local fragments = {}
  local buf = ffi.new('uint8_t[?]', self.LCM_MAX_MESSAGE_SZ)
  -- Go through the message
  local fragment_id = 0
  local fragment_offset = 0
  while fragment_offset < msg_len do
    local buf_pos = 0
    -- TODO: Use a struct here
    -- Push in the header information
    encode_u32(buf + buf_pos, MAGIC_LCM3)
    buf_pos = buf_pos + 4
    encode_u32(buf + buf_pos, msg_seq)
    buf_pos = buf_pos + 4
    encode_u32(buf + buf_pos, msg_len)
    buf_pos = buf_pos + 4
    encode_u32(buf + buf_pos, fragment_offset)
    buf_pos = buf_pos + 4
    encode_u16(buf + buf_pos, fragment_id)
    buf_pos = buf_pos + 2
    encode_u16(buf + buf_pos, fragments_in_msg)
    buf_pos = buf_pos + 2
    local n_msg_bytes_to_copy
    local n_msg_bytes_left = msg_len - fragment_offset
    if fragment_id==0 then
      -- Accrue fragment usage
      -- Copy the channel name
      ffi.copy(buf + buf_pos, channel)
      -- Plus the null terminator (OK since buffer is initially zeros)
      local fragment_usage = ch_sz + 1
      buf_pos = buf_pos + fragment_usage
      n_msg_bytes_to_copy = min(self.LCM3_MAX_FRAGMENT_SZ - fragment_usage, n_msg_bytes_left)
    else
      n_msg_bytes_to_copy = min(self.LCM3_MAX_FRAGMENT_SZ, n_msg_bytes_left)
    end
    -- Check
    -- print(fragment_id, 'n_msg_bytes_to_copy', n_msg_bytes_to_copy, buf_pos)
    ffi.copy(buf + buf_pos, msg + fragment_offset, n_msg_bytes_to_copy)
    buf_pos = buf_pos + n_msg_bytes_to_copy
    tinsert(fragments, ffi.string(buf, buf_pos))
    -- Accounting
    fragment_offset = fragment_offset + n_msg_bytes_to_copy
    fragment_id = fragment_id + 1
  end
  return fragments
end

-- TODO: channel size and message size as inputs?
local function fragment(self, channel, message, msg_sz, msg_seq)
  local ch_sz = #channel
  if ch_sz > self.LCM_MAX_CHANNEL_LENGTH then
    return false, "Channel name is too long"
  end
  if (msg_sz + ch_sz) < self.LCM2_MAX_PAYLOAD_SZ then
    return frag2(self, channel, ch_sz, message, msg_sz, msg_seq)
  else
    return frag3(self, channel, ch_sz, message, msg_sz, msg_seq)
  end
end

-- Message sender ID
function lib.gen_id(address, port)
  return bor(address, ffi.cast('uint64_t', lshift(port, 32)))
end

function lib.new_partitioning(mtu)
  local LCM_MAX_MESSAGE_SZ = partition_sizes(mtu)
  local LCM2_MAX_PAYLOAD_SZ = LCM_MAX_MESSAGE_SZ - LCM2_HEADER_SZ
  local LCM3_MAX_FRAGMENT_SZ = LCM_MAX_MESSAGE_SZ - LCM3_HEADER_SZ
  --local fragment_buffers = ffi.new('fragment_buffer[?]', LCM3_NUM_BUFFERS)
  local fragment_buffers = {
    LCM_MAX_CHANNEL_LENGTH = LCM_MAX_CHANNEL_LENGTH,
    LCM_MAX_MESSAGE_SZ = LCM_MAX_MESSAGE_SZ,
    LCM2_MAX_PAYLOAD_SZ = LCM2_MAX_PAYLOAD_SZ,
    LCM3_MAX_FRAGMENT_SZ = LCM3_MAX_FRAGMENT_SZ,
    MAX_NUM_FRAGMENTS = 2^12,
    fragment = fragment,
    assemble = assemble
  }
  return fragment_buffers
end

return lib
