#!/usr/bin/env luajit
local skt = require'skt'
local ADDRESS, PORT = "192.168.0.23", 5002
local transport = assert(skt.new_sender_receiver(ADDRESS, PORT))

local ret = transport:send('hello world')
print(ret)
