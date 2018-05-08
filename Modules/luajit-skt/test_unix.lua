#!/usr/bin/env th
local ffi = require'ffi'
local skt = require'skt'

local name = 'example'
local s0 = assert(skt.new_unix_receiver(name))

while skt.poll({s0.recv_fd}, 5e3) > 0 do
  print(s0:recv())
end
