local lib = {}

local ffi = require'ffi'
local coyield = require'coroutine'.yield
local coresume = require'coroutine'.resume
local costatus = require'coroutine'.status
local has_mmap, mmap = pcall(require, 'mmap')
if not has_mmap then io.stderr:write"No mmap support\n" end
-- local usleep = require'unix'.usleep
local utime = require'unix'.time_us
local tconcat = require'table'.concat
-- local tsort = require'table'.sort
local unpack = unpack or require'table'.unpack

-- File extension
local LOG_EXT = 'lmp' --lcm messagepack
-- Form an ISO date timestamp for filename
local iso8601_datestr = '!%Y%m%dT%H%M%SZ'
-- Find the Date string within a file
local iso8601_find = {}
for year=1,4 do table.insert(iso8601_find, '%d') end
for month=1,2 do table.insert(iso8601_find, '%d') end
for day=1,2 do table.insert(iso8601_find, '%d') end
table.insert(iso8601_find, 'T')
for hour=1,2 do table.insert(iso8601_find, '%d') end
for minutes=1,2 do table.insert(iso8601_find, '%d') end
for seconds=1,2 do table.insert(iso8601_find, '%d') end
table.insert(iso8601_find, 'Z')
iso8601_find = tconcat(iso8601_find)
-- Match a prefix, date within a logfile name
local iso8601_match = tconcat{
  "(%w-)", "[_]?", "(", iso8601_find, ")", "%.", LOG_EXT, "$"
}
-- Export
lib.iso8601_find = iso8601_find
lib.iso8601_match = iso8601_match
lib.iso8601_datestr = iso8601_datestr

-- LCM log entry header format
ffi.cdef[[
struct lcm_hdr_t {
  uint32_t sync; // LCM Sync Word
  uint64_t count; // Event Number
  uint64_t t; // Timestamp of the entry, in micro seconds
  uint32_t sz_channel; // Channel Length
  uint32_t sz_data; // Data Length
} __attribute__((packed));
]]
local lcm_hdr_t = ffi.typeof"struct lcm_hdr_t"
local lcm_hdr_sz = ffi.sizeof"struct lcm_hdr_t"
local lcm_hdr_ptr = ffi.typeof"struct lcm_hdr_t *"

local identity = function(x) return x end
local bswap = identity
if ffi.abi'le' then
  local has_bit, bit = pcall(require, 'bit')
  if has_bit then
    bswap = bit.bswap
  else
    io.stderr:write("WARNING: Logging in Little Endian mode!")
  end
end
local SYNC_WORD = bswap(0xEDA1DA01)

-- Two different MessagePack implementations...
local encode, decode
do
  local has_mp, mp = pcall(require, 'MessagePack')
  local has_mp_lj, mp_lj = pcall(require, 'msgpack_pure')
  if has_mp then
    -- io.stderr:write"Using MessagePack\n"
    if not pcall(mp.set_string, 'binaryish') then
      io.stderr:write"Using pure binary\n"
      mp.set_string'binary'
    end
    encode = mp.pack
    decode = mp.unpack
  elseif has_mp_lj then
    -- io.stderr:write"Using msgpack_pure\n"
    encode = mp_lj.pack
    decode = function(str)
      local _, obj = mp_lj.unpack(str)
      return obj
    end
  else
    io.stderr:write"msgpack not available\n"
  end
end
-- Export
lib.decode = decode
lib.encode = encode

-- Do not write the index on the fly. This can be done in post
local function write(self, obj, channel)
  local t_us = utime()
  local str = encode(obj)
  if not str then return false, "Cannot encode" end
  channel = type(channel) == 'string' and channel or self.channel
  local count = self.n_entries_log
  local hdr = lcm_hdr_t{
    SYNC_WORD,
    bswap(count),
    bswap(t_us),
    bswap(#channel),
    bswap(#str)
  }
  local entry = tconcat{
    ffi.string(hdr, lcm_hdr_sz),
    channel,
    str
  }
  -- TODO: Add ftell for retrying
  -- TODO: Check the return code of write
  local ret, err = self.f_log:write(entry)
  if not ret then return false, err end
  self.last_count = count
  self.n_entries_log = self.n_entries_log + 1
  return str, count, t_us
end

-- This is useful for log rewriting
local function write_raw(self, str, channel, t_us, count)
  channel = channel or self.channel
  t_us = t_us or utime()
  count = count or self.n_entries_log
  if type(str) ~='string' then
    return false, "Requires a string payload"
  elseif type(channel) ~='string' then
    return false, "Bad channel name"
  elseif not (type(t_us)=='number' or ffi.istype('uint64_t', t_us)) then
    return false, "Bad timestamp type"
  elseif not (type(count)=='number' or ffi.istype('uint64_t', count)) then
    return false, "Bad count type"
  elseif count < self.last_count then
    return false, "Decreasing log count"
  end
  local hdr = lcm_hdr_t{
    SYNC_WORD,
    bswap(count),
    bswap(t_us),
    bswap(#channel),
    bswap(#str)
  }
  local entry = tconcat{
    ffi.string(hdr, lcm_hdr_sz),
    channel,
    str
  }
  -- TODO: Add ftell
  local ret, err = self.f_log:write(entry)
  if not ret then
    return false, err
  end
  self.last_count = count
  self.n_entries_log = self.n_entries_log + 1
  return str
end

-- Meant to be used as a coroutine
local function playback(logname)
  local f_log = assert(io.open(logname))
  local lcm_hdr = ffi.new(lcm_hdr_t)
  coyield()
  -- Read until the end of the file
  while f_log:read(0) do
    -- Read the LCM header
    local lcm_hdr_str = f_log:read(lcm_hdr_sz)
    ffi.copy(lcm_hdr, lcm_hdr_str, lcm_hdr_sz)
    local t_us = bswap(lcm_hdr.t)
    local count = bswap(lcm_hdr.count)
    local sz_channel = bswap(lcm_hdr.sz_channel)
    local sz_data = bswap(lcm_hdr.sz_data)
    -- Read logged information
    local channel_name = f_log:read(sz_channel)
    local data_str = f_log:read(sz_data)
    coyield(data_str, channel_name, t_us, count)
  end
  f_log:close()
end

-- Add an mmap version if possible
local playback_mmap
if has_mmap then
  playback_mmap = function(logname)
    local mobj = assert(mmap.open(logname))
    local ptr, sz = unpack(mobj)
    coyield()
    local offset = 0
    while offset >= 0 and offset < sz do
      -- Read the LCM header
      local lcm_hdr = ffi.cast(lcm_hdr_ptr, ptr + offset)
      assert(lcm_hdr.sync == SYNC_WORD, "Bad sync word")
      local t_us = bswap(lcm_hdr.t)
      local count = bswap(lcm_hdr.count)
      local sz_channel = bswap(lcm_hdr.sz_channel)
      local sz_data = bswap(lcm_hdr.sz_data)
      -- Read logged information
      local channel_name = ffi.string(ptr + lcm_hdr_sz, sz_channel)
      local data_str = ffi.string(ptr + lcm_hdr_sz + sz_channel, sz_data)
      local entry_sz = lcm_hdr_sz + sz_channel + sz_data
      -- Increment pointers
      offset = coyield(data_str, channel_name, t_us, count) or (offset + entry_sz)
    end
    -- Explicit close, so keep it from being gc'd,
    -- which munmaps it
    mmap.close(mobj)
  end
  lib.playback_mmap = playback_mmap
end

-- Use callback
local function playback_multiple(lognames, callbacks)
  -- Initialize the states
  local states = {}
  for _, logname in ipairs(lognames) do
    local co_play = coroutine.create(playback)
    if coresume(co_play, logname) and costatus(co_play)=='suspended' then
      local ok, str, ch, t_us, cnt = coresume(co_play)
      if ok and costatus(co_play)=='suspended' then
        table.insert(states, {co_play, str, ch, t_us, cnt})
      end
    end
  end
  -- Initial yield is the nuber of states
  coyield(#states)
  -- Run until no logs are available
  while #states > 0 do
    -- Find the minimum timestamp
    local imin, tmin = 0, math.huge
    for i,s in ipairs(states) do
      local t_us = tonumber(s[4])
      if t_us < tmin then
        tmin = t_us
        imin = i
      end
    end
    -- Yield the minimum
    local s = states[imin]
    if not s then break end
    local str, ch, t_us, cnt = unpack(s, 2)
    if callbacks then
      local obj = decode(str)
      local fn = callbacks[ch]
      if fn then fn(obj, t_us) end
    end
    coyield(str, ch, t_us, cnt)
    -- Repopulate or remove
    local co_min = s[1]
    local ok
    ok, str, ch, t_us, cnt = coresume(co_min)
    if ok and costatus(co_min)=='suspended' then
      states[imin] = {co_min, str, ch, t_us, cnt}
    else
      table.remove(states, imin)
    end
  end
  return
end

-- Playback the logs
function lib.play(lognames, use_iterator, callbacks)
  local coro
  if type(lognames)=='string' then
    coro = playback
  elseif type(lognames)=='table' then
    coro = playback_multiple
  else
    return false, string.format("Bad lognames: %s", type(lognames))
  end
  if use_iterator then -- Wrap is a function
    local fn_play = coroutine.wrap(coro)
    local status = fn_play(lognames, callbacks)
    return fn_play, status
  else
    local co_play = coroutine.create(coro)
    local ok, status = coresume(co_play, lognames, callbacks)
    return ok and co_play, status
  end
end

local function combine(log, lognames)
  local co_play = coroutine.wrap(playback_multiple, true)
  -- local ok, status = coresume(co_play, lognames)
  -- if not ok then return false, status end
  co_play(lognames)
  for str, channel, t_us, count in co_play do
    assert(log:write_raw(str, channel, t_us))
  end
  return log
end
lib.combine = combine

local function close(self)
  if io.type(self.f_log)=='file' then
    self.f_log:close()
    -- Make read-only
    os.execute(string.format(
      "chmod a-wx '%s'", self.fname))
  end
  return self
end

local mt = {
  __gc = close
}

-- TODO: Add log rotation, a la LCM
function lib.new(name, log_dir, datestamp)
  -- Establish directory
  log_dir = type(log_dir)=='string' and log_dir or './logs'
  local status = os.execute("mkdir -p "..log_dir)
  if not status or status==0 then
    return false, string.format("Cannot make directory %s", log_dir)
  end
  -- Log new data
  if type(datestamp)~='string' or datestamp:find(iso8601_find)~=1 then
    datestamp = os.date(iso8601_datestr)
  end
  local fname = string.format("%s/%s_%s.%s",
    log_dir, name, datestamp, LOG_EXT)
  local f_log = io.open(fname, "w")
  if not f_log then
    return false, "Cannot open log file"
  end
  return setmetatable({
    write = encode and write,
    write_raw = write_raw,
    close = close,
    combine = combine,
    --
    channel = name,
    datestamp = datestamp,
    fname = fname,
    f_log = f_log,
    n_entries_log = ffi.cast('uint64_t', 0),
    last_count = ffi.cast('uint64_t', 0)
  }, mt)
end

return lib
