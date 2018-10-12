#!/usr/bin/env luajit
local lcm_packet = require'lcm_packet'
local pkt_partioner = lcm_packet.new_partitioning('localhost')
local has_skt, skt = pcall(require, 'skt')
local transport
if has_skt then
  local LCM_ADDRESS, LCM_PORT = "239.255.76.67", 7667
  transport = assert(skt.open{
      address = LCM_ADDRESS,
      port = LCM_PORT,
      ttl = 0
    })
end

-- Send "world" on the "Hello" channel
local ch0 = 'Hello'
local msg2 = 'world'
local USE_NULL_TERMINATED_STRINGS = false
if USE_NULL_TERMINATED_STRINGS then
  msg2 = msg2..string.char(0)
end
local msg_seq = 0
local fragments = assert(pkt_partioner:fragment(ch0, msg2, #msg2, msg_seq))
if transport then assert(transport:send(fragments)) end
assert(type(fragments) == 'string', "LCM 2 packet...")
local ch, msg, msg_len = assert(pkt_partioner:assemble(fragments,
                                                       #fragments))
assert(ch==ch0, string.format("Bad channel decode: [%s]/[%s]", tostring(ch):sub(1, #ch0+1), ch0))
assert(msg_len==msg_len, "Bad message decode")

-- Get a large message
local ch3 = 'Random'
local msg3 = io.open('/dev/random'):read(pkt_partioner.LCM2_MAX_PAYLOAD_SZ * 2)
local fragments3 = assert(pkt_partioner:fragment(ch3, msg3, #msg3, msg_seq + 1))
assert(type(fragments3) == 'table', "LCM 3 fragments...")
print("Partitioned", #fragments3)
local ch, msg
for ifrag, frag in ipairs(fragments3) do
  print('ifrag', ifrag)
  ch, msg = assert(pkt_partioner:assemble(frag, #frag))
end
assert(ch==ch3, "Bad channel decode ")

if transport then
  assert(transport:send_all(fragments3))
end