#!/usr/bin/env luajit
local skt = require'skt'
local ADDRESS, PORT = "239.255.65.56", 6556
local transport = assert(skt.init_mc(ADDRESS, PORT))

transport:send('hello world')
