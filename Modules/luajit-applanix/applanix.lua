local lib = {}
local ffi = require'ffi'
local C = ffi.C
local bit = require'bit'
local skt = require'skt'

local APPLANIX_ADDRESS = '192.168.53.100'
-- tcpdump -i any src host 192.168.53.100

local DISPLAY_PORT = 5600
local CONTROL_PORT = 5601
local REALTIME_PORT = 5602
local LOGGING_PORT = 5603

ffi.cdef[[
void * memmove(void *dst, const void *src, size_t len);
]]

-- Header
ffi.cdef[[
typedef struct applanix_hdr {
char start[4];
uint16_t id;
uint16_t byte_count;
double time1;
double time2;
double distance;
uint8_t time_types;
uint8_t distance_type;
} __attribute__((packed)) applanix_hdr_t;
]]
-- Footer
ffi.cdef[[
typedef struct applanix_ftr {
uint16_t checksum;
uint16_t stop;
} __attribute__((packed)) applanix_ftr_t;
]]

-- PPS
ffi.cdef[[
typedef struct pps {
applanix_hdr_t hdr;
uint32_t pps_count;
uint8_t time_sync_status;
uint8_t pad;
applanix_ftr_t ftr;
} __attribute__((packed)) pps_t;
]]

-- DMI
ffi.cdef[[
typedef struct dmi {
applanix_hdr_t hdr;
double signed_distance;
double unsigned_distance;
uint16_t scale_factor;
uint8_t status;
uint8_t type;
uint8_t rate;
uint8_t pad;
applanix_ftr_t ftr;
} __attribute__((packed)) dmi_t;
]]

-- Navigation Metrics
ffi.cdef[[
typedef struct nav_metrics {
applanix_hdr_t hdr;
float errors[12];
uint8_t pad[2];
applanix_ftr_t ftr;
} __attribute__((packed)) nav_metrics_t;
]]

-- Navigation Solution
ffi.cdef[[
typedef struct nav_solution {
applanix_hdr_t hdr;
//--
double latitude;
double longitude;
double altitude;
//--
float vnorth;
float veast;
float vdown;
//--
double roll;
double pitch;
double heading;
//--
double wander;
float track;
//
float speed;
//--
float ang_rate_long;
float ang_rate_trans;
float ang_rate_down;
//--
float acc_long;
float acc_trans;
float acc_down;
//--
uint8_t alignment;
//--
uint8_t pad[1];
applanix_ftr_t ftr;
} __attribute__((packed)) nav_solution_t;
]]

-- TODO:
-- local msg_id = {}

local grp_id = {}
grp_id[1] = function(str)
  local data = ffi.cast('nav_solution_t*', str)
  return 'Nav', data
end
grp_id[2] = function(str)
  local data = ffi.cast('nav_metrics_t*', str)
  return 'NavMetrics', data
end
grp_id[7] = function(str)
  local data = ffi.cast('pps_t*', str)
  return 'PPS', data
end
grp_id[15] = function(str)
  local data = ffi.cast('dmi_t*', str)
  return 'DMI', data
end
grp_id[30] = function(str)
  return 'EV3', str
end
grp_id[31] = function(str)
  return 'EV4', str
end
grp_id[32] = function(str)
  return 'EV5', str
end
grp_id[33] = function(str)
  return 'EV6', str
end
grp_id[20] = function()
  return 'IIN'
end
grp_id[21] = function()
  return'Base 1 GNSS'
end
grp_id[22] = function()
  return'Base 2 GNSS'
end
grp_id[23] = function()
  return'Aux 1 GNSS'
end
grp_id[24] = function()
  return'Aux 2 GNSS'
end

-- 4k buffer
local BUFFER_SZ = 4096
local buf_rt = ffi.new('uint8_t[?]', BUFFER_SZ)
local idx_rt = 0
local display_skt, realtime_skt
-- TODO:
-- local control_skt, logging_skt

local function str2short(str)
  return ffi.cast('uint16_t*', str)[0]
end

local function process_realtime(str)
  if type(str)~='string' then
    return false, "Invalid input"
  end
  local new_idx = idx_rt + #str
  if #str>BUFFER_SZ then
    io.stderr:write('BUFFER_SZ TOO SMALL\n')
    idx_rt = 0
  elseif new_idx>=BUFFER_SZ then
    new_idx = 0
  end
  ffi.copy(buf_rt + idx_rt, str)
  idx_rt = new_idx

  local idx = 0
  local remaining = idx_rt - idx
  repeat
    local hdr = ffi.cast('applanix_hdr_t*', buf_rt+idx)
    local fn = grp_id[hdr.id]
    local sz = hdr.byte_count + 8
    idx = idx + sz
    remaining = idx_rt - idx
    if type(fn)=='function' then
      local ch, data = fn(ffi.string(hdr, sz))
      coroutine.yield(ch, data)
    end
  until remaining < sz
  if idx_rt > idx then
    C.memmove(buf_rt, buf_rt + idx, idx_rt - idx)
  end
  idx_rt = 0
end

local function process_display(str)
  if type(str)~='string' then
    return false, "Invalid input"
  end
  local start = str:find'$MSG'
  if not start then return false, "No start detected" end
  local stop = str:find'$#'
  local checksum = stop and str2short(str:sub(stop-2, stop-1))
  --
  local id = str2short(str:sub(start+4, start+5))
  local byte_count = str2short(str:sub(start+6, start+7))
  local transaction = str2short(str:sub(start+8, start+9))
  --
  return transaction
end

local function update()
  local ret, ids = skt.poll({display_skt.recv_fd, realtime_skt.recv_fd}, 1e3)
  local realtime, display
  if ids[1] then
    -- display
    display = process_display(display_skt:recv())
  end
  if ids[2] then
    -- realtime
    local id, data = process_realtime(realtime_skt:recv())
  end
end

-- Single applanix, single thread...
function lib.init(_ADDRESS)
  APPLANIX_ADDRESS = _ADDRESS or APPLANIX_ADDRESS
  display_skt = assert(skt.new_sender_receiver(
                        APPLANIX_ADDRESS,
                        DISPLAY_PORT,
                        false))
  realtime_skt = assert(skt.new_stream_receiver(
                          APPLANIX_ADDRESS,
                          REALTIME_PORT))
  -- TODO:
  -- control_skt = skt.new_sender_receiver(APPLANIX_ADDRESS, CONTROL_PORT, false)
  -- logging_skt = skt.new_sender_receiver(APPLANIX_ADDRESS, LOGGING_PORT, false)
  return coroutine.wrap(function()
    local running = true
    while running do update() end
  end)
end

return lib
