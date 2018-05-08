#!/usr/bin/env luajit

print("Opening library...")
local skt = require'skt'
local ADDRESS, PORT = "239.255.65.56", 6556
print("Opening socket...")
local transport = assert(skt.open{
  address = "239.255.65.56",
  port = 6556
})
print("\n=================")
print("Opened", transport)
for k, v in pairs(transport) do
  print(k, v)
end
print("=================\n")

local ret = transport:send'hello world'
print("Sent", ret)

local msg, addr, port
repeat
  skt.poll({transport.fd}, 500)
  msg, addr, port = transport:recv()
until msg

print('Received:', msg, transport.recv_count)
