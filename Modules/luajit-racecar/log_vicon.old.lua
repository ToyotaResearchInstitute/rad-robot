#!/usr/bin/env luajit
local ffi = require'ffi'
local racecar = require'racecar'
local flags = racecar.parse_arg(arg)
local jitter_tbl = racecar.jitter_tbl
local log_announce = racecar.log_announce

local time = require'unix'.time
local poll = require'unix'.poll
local logger = require'logger'

local skt = require'skt'
local PORT = 51001
local transport = assert(skt.open{port=PORT,use_connect=false})

local RAD_TO_DEG = 180/math.pi

for k, v in pairs(transport) do
  print(k, v)
end

local log_dir = (os.getenv"RACECAR_HOME" or '.').."/logs"
local log = flags.log~=0 and assert(logger.new('vicon', log_dir))

local function exit()
  if log then log:close() end
  transport:close()
end
racecar.handle_shutdown(exit)

ffi.cdef[[
typedef struct vicon_item {
  uint8_t id;
  uint16_t sz;
  char name[24];
  double translation[3]; // mm
  double rotation[3];
} __attribute__((packed)) vicon_item;
]]

local pkt, ret
local n = 0
local blocks = {}
local t_debug = time()
while racecar.running do
  print('polling...')
  pkt = nil
  ret = skt.poll({transport.fd}, 5e3)
  -- ret = skt.poll({transport.recv_fd}, 5e3)
  while ret and ret>0 do
    -- print('servicing...')
    pkt = transport:recv()
    local t_read = time()
    if pkt then
      -- print('sz', #pkt)
      -- print('packet:', pkt:byte(1,-1))
      -- print('Counter:', transport.counter)
      local hdr = pkt:sub(1, 8)
      local frame = ffi.cast('uint32_t*', pkt:sub(1,4))[0]
      local n_items = hdr:byte(5,5)
      blocks.frame = frame
      local a, b = 6, 6+75-1
      for _=1,n_items do
        local data = pkt:sub(a, b)
        local block = ffi.cast('vicon_item*', data)
        local name = ffi.string(block.name)
        local trans, rot = {}, {}
        for ii=0,2 do
          table.insert(trans, block.translation[ii])
          table.insert(rot, block.rotation[ii])
        end
        blocks[name] = {
          translation=trans,
          rotation=rot
        }
        a = a + 75
        b = b + 75
      end
      log_announce(log, blocks, "vicon")
      local dt_debug = t_read - t_debug
      if dt_debug > 1 then
        for name, blk in pairs(blocks) do
          if name~='frame' then
            print(string.format("ID [%s] {%.2f, %.2f, %.2f} {%.2f°, %.2f°, %.2f°}",
                          name,
                          blk.translation[1], blk.translation[2], blk.translation[3],
                          blk.rotation[1]*RAD_TO_DEG, blk.rotation[2]*RAD_TO_DEG, blk.rotation[3]*RAD_TO_DEG))
          end
        end
        local info = jitter_tbl()
        io.write(table.concat(info, '\n'), '\n')
        t_debug = t_read
      end
    end
  end
end
print("done", ret)
