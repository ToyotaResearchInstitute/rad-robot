-- AUTHOR: Stephen McGill, 2017
-- Description: Decode CAN messages from ethernet frames captured with the PEAK CAN-Ethernet device

local lib = {}

------------------
-- Dependencies
local ffi = require'ffi'
local is_le = ffi.abi'le'
local bswap = require'bit'.bswap
local band = require'bit'.band
------------------

-- This assumes a Lua string
function lib.decode0(pkt)
  -- Timestamp is unused, currently
  --[[
  local ts_low = pkt:sub(13, 16)
  local ts_high = pkt:sub(17, 20)
  --]]

  -- Data Length Count (DLC)
  local dlc = pkt:byte(22)
  -- CAN ID, without flags
  local id = pkt:byte(28) + 256*pkt:byte(27) + 65536*pkt:byte(26)
  -- CAN message
  local msg = pkt:sub(29, 29 + dlc - 1)
  return id, msg
end

-- This assumes a char* from the LuaJIT ffi
function lib.decode(pkt, pkt_len)
  if pkt_len~=36 then return false, "Bad length" end
  -- Timestamp is unused, currently
  --[[
  local ts = ffi.cast('uint32_t*', pkt+12)
  local ts_high = is_le and bswap(ts[0]) or ts[0]
  local ts_low = is_le and bswap(ts[1]) or ts[1]
  --]]
  -- Data Length Count (DLC)
  local dlc = pkt[21]
  -- CAN ID and its flags
  -- local rtr = band(pkt[24], 0x40)~=0
  -- local ext = band(pkt[24], 0x80)~=0
  --local err = band(pkt[24], 0x20)~=0
  -- Form the CAN ID
  local id = ffi.cast('uint32_t*', pkt+24)[0]
  id = is_le and band(bswap(id), 0x1FFFFFFF) or band(id, 0x1FFFFFFF)
  -- CAN message
  local msg = pkt + 28
  return id, msg, dlc
end

-- Encode a message to send across the bus
local preamble = table.concat{
  string.char(0, 0x24), -- Length of Packet is 36 (0x24)
  string.char(0, 0x80), -- CAN data frame message type
  string.char(0, 1, 2, 3, 4, 5, 6, 7), -- 8 Unused "tag" data
  string.char(0, 1, 2, 3, 4, 5, 6, 7),-- TODO: Timestamp of 4 low and 4 high bytes
  string.char(3), -- Channel (unused)
}
ffi.cdef[[
typedef struct pcan {
  char preamble[21];
  uint8_t dlc;
  uint16_t flags;
  uint32_t id;
  uint8_t msg[8];
} pcan;
]]

-- This returns a Lua string
function lib.encode0(id, msg)
  local pkt = ffi.new('pcan', {preamble, #msg, 0, is_le and bswap(id) or id, msg})
  return ffi.string(pkt, 36)
end

-- This returns cdata
function lib.encode(id, msg)
  local pkt = ffi.new('pcan', {preamble, #msg, 0, is_le and bswap(id) or id, msg})
  --return pkt, ffi.sizeof'pcan'
  return pkt, 36
end

return lib