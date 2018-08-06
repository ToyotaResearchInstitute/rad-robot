local lib = {
  BASE_PORT = 51001
}

local ffi = require'ffi'

ffi.cdef[[
typedef struct vicon_item {
  uint8_t id;
  uint16_t sz;
  char name[24];
  double translation[3]; // mm
  double rotation[3]; // radians
} __attribute__((packed)) vicon_item;
]]

local item_len = 75
assert(item_len==ffi.sizeof"vicon_item", "Bad struct length")

local function parse(pkt, len)
  local is_str = type(pkt)=='string'
  local n = is_str and #pkt or len
  if n~=256 and n~=512 and n~=1024 then
    return false, "Bad vicon packet length"
  end
  local frame = is_str and pkt:sub(1, 4) or pkt
  frame = ffi.cast('uint32_t*', frame)[0]
  local n_items = is_str and pkt:byte(5) or pkt[4]
  local obj = {
    frame = frame
  }
  local str_items = is_str and pkt:sub(6) or pkt + 5
  -- TODO: If needing more packets...?
  if (n - 5) < item_len*n_items then
    return false, "Not enough data"
  end
  local items = ffi.cast('vicon_item*', str_items)
  for id=0,n_items-1 do
    local item = items[id]
    local name = ffi.string(item.name)
    local trans = {
      item.translation[0], item.translation[1], item.translation[2]
    }
    local rot = {
      item.rotation[0], item.rotation[1], item.rotation[2]
    }
    obj[name] = {
      translation=trans,
      rotation=rot
    }
  end
  return obj
end
lib.parse = parse

-- Coroutine prototype
function lib.update(data_str)
  while true do
    local obj = type(data_str)=="string" and parse(data_str)
    data_str = coroutine.yield(obj)
  end
end

return lib
