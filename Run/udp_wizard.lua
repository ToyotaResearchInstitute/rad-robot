#!/usr/bin/env luajit
dofile'../include.lua'
local Config = require'Config'
local si = require'simple_ipc'

if HOSTNAME=='alvin' or HOSTNAME=='teddy' then
	
	local s = si.new_subscriber(Config.net.test.tcp, Config.net.operator.wired)
	s.callback = function()
		local data = skt:recv_all()
		for _, tping in ipairs(data) do
			vcm.set_network_tgood(tping)
		end
		
	end
	poller:start()
	
	-- Don't forward if a robot
	os.exit()
end

local poller, lut
local in_channels = {}
local out_channels = {}
local function cb(skt)
	local ch_id = lut[skt]
	local in_ch = in_channels[ch_id]
	local out_ch = out_channels[ch_id]
	while in_ch:size() > 0 do out_ch:send(in_ch:receive()) end
end

for key,stream in pairs(Config.net.streams) do
	if type(stream.udp)=='number' then
		io.write('Forwarding ', key, ': ', stream.udp, ' -> ', stream.sub, '\n')
		local r = si.new_receiver(stream.udp)
		r.callback = cb
		table.insert(in_channels, r)
		local s = si.new_publisher(stream.sub)
		table.insert(out_channels, s)
	end
end

-- Forward the ping test packets back to the robot
do
	local r = si.new_receiver(Config.net.test.udp)
	r.callback = cb
	table.insert(in_channels, r)
	local s = si.new_publisher(Config.net.test.tcp)
	table.insert(out_channels, r)
end

poller = si.wait_on_channels(in_channels)
lut = poller.lut
poller:start()