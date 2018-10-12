#!/usr/bin/env luajit

local ttyname = arg[1] or "/dev/ttyACM0"
local val = tonumber(arg[2]) or false

local fw_get_pkt = string.char(2,1,0,0,0,3)
print("Get firmware Packet")
print(fw_get_pkt:byte(1, -1))

local vesc = require'vesc'
print("Firmware", unpack(vesc.firmware_version()))

print("SET packet to", val)

-- local send_pkt = assert(vesc.current(val or 0))
-- local send_pkt = assert(vesc.servo_position(val or 0.5))
-- local send_pkt = assert(vesc.duty_cycle(val or 0))
local send_pkt = assert(vesc.sensors())

local ptbl = {}
for i = 1, #send_pkt do
  table.insert(ptbl, string.format("0x%02X", send_pkt[i]))
end
print(table.concat(ptbl, " "))
print()


local has_unix, unix = pcall(require, 'unix')
if not has_unix then
  print("No unix module for opening serial port")
  os.exit()
end

local has_stty, stty = pcall(require, 'stty')
if not has_stty then
  print("No stty module for setting serial port")
  os.exit()
end

local fd = unix.open(ttyname, unix.O_RDWR + unix.O_NOCTTY + unix.O_NONBLOCK)
assert(fd > 0, "Bad File descriptor")

stty.raw(fd)
stty.serial(fd)
stty.speed(fd, 115200)

-- Get the firmware
print("Getting the firmware from the board")
stty.flush(fd)
unix.write(fd, fw_get_pkt)
stty.drain(fd)

local coro_vesc = coroutine.create(vesc.update)
while unix.select({fd}, 0.1) > 0 do
  local data = unix.read(fd)
  print("Read", data and #data)
  local status, obj, msg = coroutine.resume(coro_vesc, data)
  print("Coro", status, obj, msg)
  while status and obj do
    if obj then
      print("Version:", obj)
      break
    end
    status, obj, msg = coroutine.resume(coro_vesc)
    print("Coro", status, obj, msg)
  end
end

print("Sending command the board")

-- local send_pkt = assert(vesc.current(val or 0))
-- local send_pkt = assert(vesc.servo_position(val or 0.5))
local send_pkts = {
  assert(vesc.sensors()),
  --assert(vesc.duty_cycle(val or 0)),
  assert(vesc.rpm(val or 0)),
  assert(vesc.sensors()),
  assert(vesc.rpm(val or 0)),
  assert(vesc.sensors())
}

for i, send_pkt in ipairs(send_pkts) do
  io.stderr:write("Sending ", i, "\n")
  stty.flush(fd)
  unix.write(fd, string.char(unpack(send_pkt)))
  stty.drain(fd)

  -- Poll the sensors only
  if i%2==1 then
    while unix.select({fd}, 0.1) > 0 do
      local data = unix.read(fd)
      status, obj, pkt_state = coroutine.resume(coro_vesc, data)
      while status and obj do
        for k, v in pairs(obj) do
          if type(v) == 'table' then
            print(k, unpack(v))
          else
            print(k, v)
          end
        end
        status, obj, msg = coroutine.resume(coro_vesc)
      end
    end
    --[[
    if type(pkt_data)=='table' then
      print("ID: ", pkt_data[1])
      if pkt_data[1]==4 then
        local values = vesc.parse_values(pkt_data)
        for k, v in pairs(values) do
          if type(v) == 'table' then
            print(k, unpack(v))
          else
            print(k, v)
          end
        end
      else
        print(unpack(pkt_data, 2))
      end
    end -- if data
    --]]
  end

end

unix.close(fd)
