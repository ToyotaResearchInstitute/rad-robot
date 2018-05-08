#!/usr/bin/env luajit
local skt = require'skt'
local PORT = tonumber(arg[1]) or 5002
local transport = assert(skt.open{port=PORT,use_connect=false})

for k, v in pairs(transport) do
  print(k, v)
end

local pkt, ret
repeat
  print('polling...')
  pkt = nil
  ret = skt.poll({transport.fd}, 5e3)
  while ret and ret>0 do
    print('servicing...')
    pkt = transport:recv()
    if pkt then
      print('sz', #pkt)
      print('packet:', pkt:byte(1,-1))
      print('Counter:', transport.counter)
    else
      print("no pkt...")
    end
    ret = skt.poll({transport.recv_fd}, 0) -- immediate
  end
until false and not pkt
print("done", ret)
