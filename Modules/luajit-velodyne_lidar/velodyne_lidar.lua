local lib = {}

local ffi = require'ffi'
local nmea = require'nmea'
local tinsert = require'table'.insert
local eps = 1e-23
local fabs = require'math'.abs

ffi.cdef[[
typedef struct laser_return {
  uint16_t distance; // 2mm
  uint8_t intensity; // 0 is no return up to 65 meters
} __attribute__((packed)) laser_return;

typedef struct velodyne_block {
  uint8_t flag[2];
  uint16_t azimuth; // Units of 1/100 of a degree
  laser_return returnsA[16]; // VLP16
  laser_return returnsB[16];
} __attribute__((packed)) velodyne_block;

typedef struct velodyne_data {
  velodyne_block blocks[12];
  uint32_t gps_timestamp;
  uint8_t return_mode;
  uint8_t model;
} __attribute__((packed)) velodyne_data;

typedef struct velodyne_position {
  uint8_t unused[198];
  uint32_t gps_timestamp;
  uint32_t blank;
  uint8_t nmea[72];
  uint8_t pad[234];
} __attribute__((packed)) velodyne_position;
]]

-- Polar elevation angles
local polar = {
  -15, 1, -13, -3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15
}
-- Convert to radians
for i=1,#polar  do
  polar[i] = polar[i] * math.pi / 180
end
local sin = require'math'.sin
local cos = require'math'.cos
local function to_xyz(ds, azi)
  local xs, ys, zs = {}, {}, {}
  for i, d in ipairs(ds) do
    local p = polar[i]
    local a = d * cos(p)
    local x, y, z = a * sin(azi), a * cos(azi), d * sin(p)
    xs[i], ys[i], zs[i] = x, y, z
  end
  return xs, ys, zs
end
lib.to_xyz = to_xyz
local function xyz_packed(distances, azimuths, pts, cur)
  pts = pts or {}
  cur = cur or (#pts + 1)
  for j, azi in ipairs(azimuths) do
    for i, d in ipairs(distances[j]) do
      local p = polar[i]
      local a = d * cos(p)
      local x, y, z = a * sin(azi), a * cos(azi), d * sin(p)
      pts[cur] = x
      pts[cur+1] = y
      pts[cur+2] = z
      cur = cur + 3
    end
  end
  return pts, cur
end
lib.xyz_packed = xyz_packed

local function xyz_cb(distances, azimuths, cb)
  if type(cb)~='function' then return end
  local n = 0
  for j, azi in ipairs(azimuths) do
    for i, d in ipairs(distances[j]) do
      local p = polar[i]
      if d > eps then
        local a = d * cos(p)
        local x, y, z = a * sin(azi), a * cos(azi), d * sin(p)
        cb{x, y, z}
        n = n + 1
      end
    end
  end
  return n
end
lib.xyz_cb = xyz_cb

local function intensities_planar(intensities, is, cur)
  is = is or {}
  cur = cur or (#is + 1)
  for _, int0 in ipairs(intensities) do
    for _, int in ipairs(int0) do
      is[cur] = int
      cur = cur + 1
    end
  end
  return is, cur
end
lib.intensities_planar = intensities_planar

local function xyz_planar(distances, azimuths)
  local xs, ys, zs = {}, {}, {}
  for j, azi in ipairs(azimuths) do
    for i, d in ipairs(distances[j]) do
      local p = polar[i]
      local a = d * cos(p)
      local x, y, z = a * sin(azi), a * cos(azi), d * sin(p)
      tinsert(xs, x)
      tinsert(ys, y)
      tinsert(zs, z)
    end
  end
  return xs, ys, zs
end
lib.xyz_planar = xyz_planar

-- Raw packet, less the 42 byte UDP header
local VELO_DATA_SZ = 1248 - 42
local VELO_POSITION_SZ = 554 - 42

local function parse_blocks(blocks)
  -- Save the azimuth
  -- May need to bswap...
  local azi = {}
  for i=0,11 do
    local block = blocks[i]
    -- if block.flag[1]~=0xEE then end
    azi[2*i+1] = block.azimuth / 100 -- degrees
  end
  -- TODO: This should be a mod_angle average
  for i=2, 22, 2 do
    local a, c = azi[i-1], (azi[i+1] or azi[1])
    if a > c then c = c + 360 end
    local b = (a + c) / 2
    if b > 360 then b = b - 360 end
    -- azi[i] = (azi[i-1] + azi[i+1]) / 2
    azi[i] = b
  end
  -- Convert to radians
  for i=1, #azi do azi[i] = math.rad(azi[i]) end

  -- Distances and intensities
  local distances, intensities = {}, {}
  for i=0,11 do
    local block = blocks[i]
    -- if block.flag[1]~=0xEE then end
    local rA, dA, iA = block.returnsA, {}, {}
    local rB, dB, iB = block.returnsB, {}, {}
    -- VLP16
    for j=0, 15 do
      tinsert(dA, rA[j].distance * 2) -- millimeters
      tinsert(iA, rA[j].intensity)
      tinsert(dB, rB[j].distance * 2) -- millimeters
      tinsert(iB, rB[j].intensity)
    end
    tinsert(distances, dA)
    tinsert(distances, dB)
    tinsert(intensities, iA)
    tinsert(intensities, iB)
  end
  return azi, distances, intensities
end

local id2model = {
  [0x21] = 'HDL-32E',
  [0x22] = 'VLP-16',
}
lib.id2model = id2model

local function parse_data(str)
  if #str~=VELO_DATA_SZ then return false, "Bad packet length" end
  local ptr_pkt_data = ffi.cast('velodyne_data*', str)
  local blocks = ptr_pkt_data.blocks
  local azi, distances, intensities = parse_blocks(blocks)
  return {
    azi = azi, -- radians
    dist = distances, -- millimeters
    int = intensities,
    t_hr = ptr_pkt_data.gps_timestamp, -- microseconds
    mode = ptr_pkt_data.return_mode,
    model = ptr_pkt_data.model
    -- model = id2model[ptr_pkt_data.model]
  }
end
lib.parse_data = parse_data

local function parse_position(str)
  if #str~=VELO_POSITION_SZ then return false, "Bad packet length" end
  local ptr_pkt_pos = ffi.cast('velodyne_position*', str)
  local obj = {t_us = ptr_pkt_pos.gps_timestamp}
  local nmea_sentence = ffi.string(ptr_pkt_pos.nmea, 72)
  -- print("NMEA", nmea_sentence)
  obj.nmea = nmea_sentence
  nmea.parse_nmea(nmea_sentence, obj)
  return obj
end
lib.parse_position = parse_position

-- Just give the struct
function lib.update(data_str)
  while true do
    local obj = data_str and parse_data(data_str)
    data_str = coroutine.yield(obj)
  end
end

lib.POSITION_PORT = 8308
lib.DATA_PORT = 2368

return lib
