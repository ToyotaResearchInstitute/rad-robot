#!/usr/bin/env luajit
local ffi = require'ffi'
local skt = require'skt'
local ADDRESS, PORT = "239.255.65.56", 6556
local transport = assert(skt.init_mc(ADDRESS, PORT))

if ffi.os=='Linux' then
  local msgs
  repeat
    msgs = transport:recvm()
  skt.poll({}, 500)
  until msgs

  print(#msgs, transport.counter)
  for i,m in ipairs(msgs) do
    print("Packet "..i, #m[1], m[2], m[3])
  end
end

local msg, addr, port
repeat
  msg, addr, port = transport:recv()
  skt.poll({}, 500)
until msg

print('Received:', msg, transport.counter)

