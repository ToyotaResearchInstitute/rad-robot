-- Compatible with Lua 5.3+ and LuaJIT 2.1+
local lib = {}

local cocreate = require'coroutine'.create
local coresume = require'coroutine'.resume
local costatus = require'coroutine'.status
local cowrap = require'coroutine'.wrap
local coyield = require'coroutine'.yield

local M_HUGE = require'math'.huge

local sformat = require'string'.format
-- String pack introduced in Lua 5.3
local spack = require'string'.pack

local tconcat = require'table'.concat
local tinsert = require'table'.insert
local tremove = require'table'.remove
local tsort = require'table'.sort
-- table.unpack introduced in Lua 5.2; Using Luajit's 5.2 COMPAT mode, always
local unpack = unpack or require'table'.unpack

-- Special packages, depending on the version of Lua used
local has_ffi, ffi = pcall(require, 'ffi')
local has_mmap, mmap = pcall(require, 'mmap')
if not has_mmap then io.stderr:write"No mmap support\n" end

-- Cycle logs that hit a certain file size limit
-- Use the max size of a file on FAT32: https://en.wikipedia.org/wiki/File_Allocation_Table#FAT32
local MAX_FILE_SZ = 2^32 - 1

-- File extension
local LOG_EXT = 'lmp' -- LCM MessagePack
local FILENAME_FMT = "%s_%s-%03d."..LOG_EXT
-- Form an ISO date timestamp for filename
local iso8601_datestr = '!%Y%m%dT%H%M%SZ'
local iso8601_datelen = #os.date(iso8601_datestr)
-- Find the Date string within a file
local iso8601_find = {}
for year=1,4 do tinsert(iso8601_find, '%d') end
for month=1,2 do tinsert(iso8601_find, '%d') end
for day=1,2 do tinsert(iso8601_find, '%d') end
tinsert(iso8601_find, 'T')
for hour=1,2 do tinsert(iso8601_find, '%d') end
for minutes=1,2 do tinsert(iso8601_find, '%d') end
for seconds=1,2 do tinsert(iso8601_find, '%d') end
tinsert(iso8601_find, 'Z')
iso8601_find = tconcat(iso8601_find)
-- Match a prefix, date within a logfile name
-- Returns: prefix (default channel), iso8601 date, chunk id
-- Require the underscore
local lmp_match = tconcat{
  -- "^", "(%w-)", "_", "(", iso8601_find, ")", "%-", "(%d+)", "%.", LOG_EXT, "$"
  "(%w-)", "_", "(", iso8601_find, ")", "%-", "(%d+)", "%.", LOG_EXT, "$"
}
-- local lmp_match_old = tconcat{
--   "^", "(%w-)", "_", "(", iso8601_find, ")", "%.", LOG_EXT, "$"
-- }
-- Export
lib.iso8601_datestr = iso8601_datestr
lib.iso8601_datelen = iso8601_datelen
lib.iso8601_find = iso8601_find
lib.lmp_match = lmp_match

-- return math.type(v) == 'integer' or ffi.istype('uint64_t', t_us)
local identity = function(x) return x end
local bswap = identity
if has_ffi then
  if ffi.abi'le' then
    local has_bit, bit = pcall(require, 'bit')
    if has_bit then
      bswap = bit.bswap
    else
      io.stderr:write("WARNING: Logging in Little Endian mode!")
    end
  end

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
  -- TODO: Allow a non-FFI version, here
  local lcm_hdr_t = ffi.typeof"struct lcm_hdr_t"
  local LCM_HDR_SZ = ffi.sizeof"struct lcm_hdr_t"
  local lcm_hdr_ptr = ffi.typeof"struct lcm_hdr_t *"
  local SYNC_WORD = bswap(0xEDA1DA01)

  local function form_entry(str, channel, t_us, count)
    local hdr = lcm_hdr_t{
      SYNC_WORD,
      bswap(count),
      bswap(t_us),
      bswap(#channel),
      bswap(#str)
    }
    local entry = tconcat{ffi.string(hdr, LCM_HDR_SZ), channel, str}
    return entry
  end

  local function parse_header(lcm_hdr)
    assert(lcm_hdr.sync == SYNC_WORD, sformat("Bad sync word! %X", lcm_hdr.sync))
    local t_us = bswap(lcm_hdr.t)
    local count = bswap(lcm_hdr.count)
    local sz_channel = bswap(lcm_hdr.sz_channel)
    assert(sz_channel < 256, sformat("Channel length too long: [%d]", sz_channel))
    local sz_data = bswap(lcm_hdr.sz_data)
    return sz_data, sz_channel, t_us, count
  end

  local C = ffi.C
  ffi.cdef[[
  typedef struct timeval {
    long tv_sec;
    int32_t tv_usec;
  } timeval;
  int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
  ]]
  local function utime()
    local t = ffi.new'timeval'
    C.gettimeofday(t, nil)
    return 1e6 * ffi.cast('uint64_t', t.tv_sec) + t.tv_usec
  end

elseif spack then
  -- Using the string.pack/unpack methods
  -- LCM log entry header format
  local SYNC_WORD = 0xEDA1DA01
  local LCM_HDR_FMT = "> I4 I8 I8 I4 I4"
  form_entry = function (str, channel, t_us, count)
    local hdr = spack(LCM_HDR_FMT,
      SYNC_WORD,
      count,
      t_us,
      #channel,
      #str
    )
    local entry = tconcat{hdr, channel, str}
    return entry
  end

  local function parse_header(lcm_hdr)
    local sync, count, t_us, sz_channel, sz_data = sunpack(LCM_HDR_FMT, lcm_hdr)
    assert(sync == SYNC_WORD, sformat("Bad sync word! %X", sync))
    assert(sz_channel < 256, sformat("Channel length too long: [%d]", sz_channel))
    return sz_data, sz_channel, t_us, count
  end

  -- Precision in seconds... not great, but just to have some placeholder
  local otime = require'os'.time
  local function utime()
    return 1e6 * otime()
  end

end

local function check_write(self, str, channel, t_us, count)
  if type(str) ~='string' then
    return false, "Requires a string payload"
  elseif type(channel) ~='string' then
    return false, "Bad channel name"
  elseif not (type(t_us)=='number' or ffi.istype('uint64_t', t_us)) then
    return false, "Bad timestamp type"
  elseif type(count)~='number' then
    return false, "Bad count type"
  elseif count < self.last_count then
    return false, "Decreasing log count"
  elseif t_us < self.last_t_us then
    return false, "Decreasing log times"
  end
  return true
end

local function next_chunk(self)
  -- TODO: Unix call for chmod to read-only
  self.f_log:close()
  -- Update the chunk and log name
  self.chunk_id = self.chunk_id + 1
  local log_name = sformat(FILENAME_FMT, self.channel, self.datestamp, self.chunk_id)
  self.log_name = log_name
  local fname = sformat("%s/%s", self.log_dir, log_name)
  -- Use append mode if a new log
  -- TODO: Check if the file exists, and we want to overwrite (harvesting)
  local f_log = io.open(fname, "a")
  if not f_log then
    return false, "Cannot open log file"
  end
  self.f_log = f_log
  -- Reset the size
  -- self.sz_chunk = ffi.cast('uint64_t', 0)
  self.sz_chunk = 0
  io.stderr:write(sformat("Next chunk: %s\n", log_name))
  return self
end

-- This is useful for log rewriting
local function write_raw(self, str, channel, t_us, count)
  channel = channel or self.channel
  t_us = t_us or utime()
  count = count or self.n_entries
  check_write(self, str, channel, t_us, count)
  local entry = form_entry(str, channel, t_us, count)
  local sz_entry = #entry
  if self.sz_chunk + sz_entry > MAX_FILE_SZ then
    -- Must close and open a new one
    next_chunk(self)
  end
  -- TODO: Add ftell
  -- TODO: Check the return code of write
  local ret, err = self.f_log:write(entry)
  if not ret then return false, err end
  self.last_count = count
  self.last_t_us = t_us
  self.n_entries = self.n_entries + 1
  self.sz_chunk = self.sz_chunk + sz_entry
  self.sz_log = self.sz_log + sz_entry
  return str, count, t_us
end

-- Do not write the index on the fly. This can be done in post
local function write(self, obj, channel, t_us)
  t_us = t_us or utime()
  local str = encode(obj)
  if not str then return false, "Cannot encode" end
  channel = type(channel) == 'string' and channel or self.channel
  local count = self.n_entries
  return write_raw(self, str, channel, t_us, count)
end

-- Meant to be used as a coroutine; yields:
-- data_str: string or number
-- channel_name: string or number
-- t_us: ctype of uint64_t
-- count: ctype of uint64_t
-- idx: number
local function playback(self, options)
  local f_log = self.f_log
  local f_type = io.type(f_log)
  if f_type ~= 'file' then
    return false, "Bad file handle: "..tostring(f_type)
  end
  -- Get the file size
  -- TODO: Deal with streaming, which has no seek
  local sz_log, err_seek = f_log:seek("end")
  if not sz_log then
    return false, err_seek
  elseif sz_log==0 then
    return false, "Empty log"
  end

  if type(options) ~= 'table' then options = {} end
  local skip_data = options.skip_data
  local skip_channel = options.skip_channel

  -- Go back to the start of the file
  f_log:seek("set")
  -- Give the log size
  local log_info = {
    sz_log = sz_log, -- file size
    n_entries = 0,
    -- size of valid part of log (ideally same as sz_log)
    sz_entries = 0,
    corrupted = false
  }
  coyield(log_info)
  -- Grab the index of this entry
  local idx = f_log:seek()
  -- Read until the end of the file
  while f_log:read(0) do
    -- Read the LCM header
    local lcm_hdr_str = f_log:read(LCM_HDR_SZ)
    if #lcm_hdr_str ~= LCM_HDR_SZ then
      io.stderr:write("Corrupted log: Bad Header Size\n")
      log_info.corrupted = true
      break
    end
    -- TODO: Use ffi.cast?
    local lcm_hdr = ffi.new(lcm_hdr_t)
    ffi.copy(lcm_hdr, lcm_hdr_str, LCM_HDR_SZ)
    local sz_data, sz_channel, t_us, count = parse_header(lcm_hdr)
    if not sz_data then
      io.stderr:write("Corrupted log: Bad Header Parsing\n")
      log_info.corrupted = true
      break
    end
    -- Read logged information
    local channel_name
    if skip_channel then
      channel_name = sz_channel
      f_log:seek("cur", sz_channel)
    else
      channel_name = f_log:read(sz_channel)
      -- Check that enough data is recorded
      if #channel_name~=sz_channel then
        io.stderr:write("Corrupted log: Channel\n")
        log_info.corrupted = true
        break
      end
    end
    -- TODO: Make data reading optional: return data size, instead
    local data_str
    if skip_data then
      data_str = sz_data
      f_log:seek("cur", sz_data)
    else
      data_str = f_log:read(sz_data)
      -- Check that enough data is recorded
      if #data_str~=sz_data then
        io.stderr:write("Corrupted log: Payload\n")
        log_info.corrupted = true
        break
      end
    end
    log_info.sz_entries = f_log:seek()
    log_info.n_entries = log_info.n_entries + 1
    -- In case of appending after indexing
    self.last_count = count
    self.last_t_us = t_us
    coyield(data_str, channel_name, t_us, count, idx)
    -- Update the index for the next round
    idx = log_info.sz_entries
  end
  io.stderr:write("Done!\n")
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
      local cur_ptr = ptr + offset
      -- Read the LCM header
      -- local lcm_hdr = ffi.cast(lcm_hdr_ptr, cur_ptr)
      -- local sz_data, sz_channel, t_us, count = parse_header(lcm_hdr)
      -- Try the implicit cast
      local sz_data, sz_channel, t_us, count = parse_header(cur_ptr)
      -- Read logged information
      local channel_name = ffi.string(cur_ptr + LCM_HDR_SZ, sz_channel)
      local data_str = ffi.string(cur_ptr + LCM_HDR_SZ + sz_channel, sz_data)
      local entry_sz = LCM_HDR_SZ + sz_channel + sz_data
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
-- TODO: Function to find smallest in state
local function playback_multiple(logs, options)
  if type(options) ~= 'table' then options = {} end
  local callbacks = options.callbacks
  if type(callbacks)~='table' then
    callbacks = {}
  end
  -- Initialize the states
  local states = {}
  for ilog, log in ipairs(logs) do
    local co_play = cocreate(playback)
    local status0, sz_log = assert(coresume(co_play, log, options))
    local status1 = costatus(co_play)
    if status0 and status1=='suspended' then
      local ok, str, ch, t_us, cnt, idx = coresume(co_play)
      if ok and costatus(co_play)=='suspended' then
        tinsert(states, {co_play, ilog, str, ch, t_us, cnt, idx})
      end
    end
  end
  -- Initial yield is the number of states
  coyield(#states)
  -- Run until no logs are available
  while #states > 0 do
    -- Find the minimum timestamp
    local imin, tmin = 0, M_HUGE
    for i,s in ipairs(states) do
      local t_us = tonumber(s[5])
      if t_us < tmin then
        tmin = t_us
        imin = i
      end
    end
    -- Yield the minimum
    local s = states[imin]
    if not s then break end
    local str, ch, t_us, cnt, idx = unpack(s, 3)
    local fn = callbacks[ch]
    if fn then
      local obj = decode(str)
      fn(obj, t_us)
    end
    -- Yield an additional index
    local ilog_min = s[2]
    coyield(str, ch, t_us, cnt, idx, ilog_min)
    -- Repopulate or remove
    local co_min = s[1]
    local ok
    ok, str, ch, t_us, cnt, idx = coresume(co_min)
    -- Run to get the next values
    if ok and costatus(co_min)=='suspended' then
      states[imin] = {co_min, ilog_min, str, ch, t_us, cnt, idx}
    else
      tremove(states, imin)
    end
  end
  return
end

local function play(self, options)
  if type(options) ~= 'table' then options = {} end
  if options.use_iterator then -- Wrap is a function
    local fn_play = cowrap(playback)
    local sz_log = fn_play(self, options)
    return fn_play, sz_log
  else
    local co_play = cocreate(playback)
    local ok, sz_log, err = coresume(co_play, self, options)
    if not ok then
      return false, sz_log
    elseif not sz_log then
      return false, err
    end
    return co_play, sz_log
  end
end

local function play_many(logs, options)
  -- Playback the logs
  if type(options) ~= 'table' then options = {} end
  local use_iterator = options.use_iterator
  if use_iterator then -- Wrap is a function
    local fn_play = cowrap(playback_multiple)
    local sz_log = fn_play(logs, options)
    return fn_play, sz_log
  else
    local co_play = cocreate(playback_multiple)
    local ok, sz_log = coresume(co_play, logs, options)
    return ok and co_play, sz_log
  end
end
lib.play_many = play_many

-----------------
-- Log opening --
-----------------

local function close(self)
  -- Close the log
  if io.type(self.f_log)=='file' then
    self.f_log:close()
    -- Make read-only
    local fname0 = sformat("%s/%s", self.log_dir, self.log_name)
    os.execute(sformat(
      "chmod a-wx '%s'", fname0))
  end
  if io.type(self.f_idx)=='file' then
    self.f_idx:close()
    -- Make read-only
    local fname0 = sformat("%s/%s", self.log_dir, self.idx_name)
    os.execute(sformat(
      "chmod a-wx '%s'", fname0))
  end
  -- Allow for re-opening
  return self
end

local mt = {
  __gc = close,
  __index = function(t, k) return lib[k] end,
}

-- TODO: Add log rotation, a la LCM
local function new_log(ch_default, log_dir, datestamp, chunk_id)
  if type(ch_default) ~= 'string' then
    -- No default
    ch_default = ''
  end
  -- Establish directory
  log_dir = type(log_dir)=='string' and log_dir or './logs'
  local status = os.execute("mkdir -p "..log_dir)
  if not status or status==0 then
    return false, sformat("Cannot make directory %s", log_dir)
  end
  chunk_id = tonumber(chunk_id) or 0
  if chunk_id < 0 then
    return false, "Bad chunk id"
  elseif chunk_id > 1000 then
    -- Limit: No more than 1000 chunks per datestamp
    datestamp = nil
  end
  -- Log new data
  if type(datestamp)~='string' then
    datestamp = os.date(iso8601_datestr)
  elseif #datestamp~=iso8601_datelen or datestamp:find(iso8601_find)~=1 then
    io.stderr:write("Unknown datestamp\n")
  end
  local log_name = sformat(FILENAME_FMT, ch_default, datestamp, chunk_id)
  local fname = sformat("%s/%s", log_dir, log_name)
  -- Use append mode if a new log
  -- TODO: Check if the file exists, and we want to overwrite (harvesting)
  local f_log = io.open(fname, "a")
  if not f_log then
    return false, "Cannot open log file"
  end
  local f_idx = false
  return setmetatable({
    write = encode and write,
    write_raw = write_raw,
    close = close,
    --
    f_log = f_log,
    f_idx = f_idx,
    --
    log_name = log_name,
    log_dir = log_dir,
    --
    channel = ch_default,
    datestamp = datestamp,
    chunk_id = chunk_id,
    --
    -- sz_chunk = ffi.cast('uint64_t', 0),
    sz_chunk = 0,
    sz_log = 0,
    n_entries = 0,
    -- last_count = ffi.cast('uint64_t', 0),
    -- TODO: Should count span across chunks?
    last_count = 0,
    -- last_t_us = ffi.cast('uint64_t', 0),
    last_t_us = 0,
  }, mt)
end
lib.new = new_log

local function reopen_log(self)
  -- Form the filename
  local fname0 = sformat("%s/%s", self.log_dir, self.log_name)
  -- Open the file as read-only
  local f_log = io.open(fname0, "r")
  if not f_log then
    return false, "Cannot open log file"
  end
  
  return self
end

-- Open a filename
local function open_log(log_fname)
  -- Check if we already have a log object
  if type(log_fname) == 'table' then
    return reopen_log(log_fname)
  end


  -- If the pattern has captures, then in a successful match the captured values are also returned, after the two indices.
  local name_start, name_end, ch_default, datestamp, chunk_id = log_fname:find(lmp_match)
  local log_name, log_dir
  if not name_start then
    io.stderr:write"Bad log name\n"
    ch_default = ''
    datestamp = ''
  elseif name_start == 1 then
    -- Check if the file has no path component - then it lies in the same directory
    -- Drawbacks: https://vineetreddy.wordpress.com/2017/05/17/pwd-vs-getcwd/
    log_dir = os.getenv'PWD'
    log_name = log_fname
  else
    log_dir = log_fname:sub(1, name_start)
    log_name = log_fname:sub(name_start, name_end)
  end
  -- Convert the chunk id to a number
  chunk_id = tonumber(chunk_id) or 0


  -- Open the file
  local f_log = io.open(fname, "r")
  if not f_log then
    return false, "Cannot open log file"
  end
  -- TODO: Try to open an index file
  -- TODO: Open index if it exists
  local f_idx = false
  -- Return a log object
  return setmetatable({
    write = encode and write,
    write_raw = write_raw,
    close = close,
    play = play,
    -- File handles
    f_log = f_log,
    f_idx = f_idx,
    --
    idx_name = false,
    log_name = log_name,
    log_dir = log_dir,
    log_fname = log_fname,
    --
    channel = ch_default,
    datestamp = datestamp,
    chunk_id = chunk_id,
    --
    -- double values should suffice to capture the number of entries
    -- http://www.inf.puc-rio.br/~roberto/talks/ws2014.pdf
    -- doubles can count up to 2^52 positive integers
    -- MAX FILE SZ is 2^32
    -- 2^52 / 2^32 = 2^20 = 1 million
    -- No need to use ffi.cast('uint64_t', 0)
    sz_log = 0,
    n_entries = 0,
    sz_chunk = 0,
    last_count = 0,
    last_t_us = 0,
  }, mt)

end
lib.open = open_log

------------------
-- Log Indexing --
------------------

local function index(self, override)
  if self.idx_name and not override then
    -- TODO: We should simply check the index
    return false, "Log is indexed, already"
  end
  -- Suffix is .lmp.idx
  local idx_name = self.log_name..".idx"
  local fname_idx = sformat("%s/%s", self.log_dir, idx_name)
  -- Open in append mode, as we do not seek
  local f_idx = io.open(fname_idx, "a")
  if not f_idx then
    return false, "Cannot open idx file"
  end

  local options = {
    skip_data = true,
    skip_channel = true,
    use_iterator = true
  }
  for sz_msg, sz_ch, t_us, cnt, idx in self:play(options) do
    -- Index format: [entry_chunk entry_offset entry_size]
    -- NOTE: We use Big endian 32 bits for each number
    -- Can check the output with od -t u4
    -- TODO: Add string.pack
    -- Byte swapping works with 32bit number coersion
    local idx_record = ffi.new('uint32_t[3]',
                               {0, bswap(idx), bswap(LCM_HDR_SZ + sz_ch + sz_msg)})
    -- Write the record via string
    -- NOTE: Using ffi.string instead of using fwrite. fwrite may have speed, but not simplicity
    assert(f_idx:write(ffi.string(idx_record, ffi.sizeof(idx_record))))
  end
  -- TODO: Check corruption

  return self
end
lib.index = index

-------------------
-- Log utilities --
-------------------

local function to_logs(logs)
  -- In-place
  for i=1, #logs do
    if type(logs[i]) == 'string' then
      logs[i] = assert(open_log(logs[i]))
    end
  end
  return logs
end

local function combine(self, logs)
  -- Combine many
  local options = {use_iterator=true}
  local co_play = cowrap(playback_multiple, options)
  co_play(to_logs(logs))
  for str, channel, t_us in co_play do
    assert(self:write_raw(str, channel, t_us))
  end
  return self
end
lib.combine = combine

-- TODO: Function to get the channels in a log
local function get_channels(self, channels)
  local options = {
    use_iterator = true,
    skip_data = true
  }
  local it_log, sz_log = self:play(options)
  -- Store in given table, if possible
  if type(channels) ~= 'table' then channels = {} end
  for _, ch in it_log do channels[ch] = true end
  return channels
end

-- For synchronization
local function by_ts(t_usA, t_usB)
  return t_usA[3] < t_usB[3]
end

function lib.sync(self, logs, options)
  logs = to_logs(logs)
  -- Write out a synchronized log to self
  if type(options) ~= 'table' then options = {} end
  -- Regex support if given flags
  local include = type(options.include) == 'string' and options.include
  local exclude = type(options.exclude) == 'string' and options.exclude
  -- Bootup the messages by acquiring all the channels
  local channels = {}
  for _, log in ipairs(logs) do get_channels(log, channels) end
  local no_filter = not (include or exclude)
  local messages = {}
  for ch in pairs(channels) do
    if no_filter then
      messages[ch] = false
    else
      -- Make sure it isn't excluded
      local is_included = (not include) or ch:find(include) ~= nil
      local is_excluded = exclude and (ch:find(exclude) ~= nil)
      if is_included and not is_excluded then
        messages[ch] = false
      end
    end
  end
  -- Keep a list
  local included_channels = {}
  for ch in pairs(messages) do
    tinsert(included_channels, ch)
  end
  tsort(included_channels)
  -- Set the maximum sample rate
  local tp_sample = type(options.dt_sample)
  local dt_us_sample
  if tp_sample=='number' then
    -- Use microseconds, the format of the log timestamps
    dt_us_sample = options.dt_sample * 1e6
  elseif tp_sample=='string' and messages[options.dt_sample]~=nil then
    -- Use a message name as the synchronization (must be in messages)
    dt_us_sample = options.dt_sample
  else
    return false, sformat("Bad dt_sample [%s]: [%s]",
      tostring(options.dt_sample), table.concat(included_channels, ", "))
  end
  -- Log format: Cannot retain old messages (default)
  -- Object format: Retain old messages for non-unique data
  local use_log_fmt = not options.use_obj_fmt
  -- How to handle the synchronization
  local handler
  if type(self) == 'table' then
    if use_log_fmt then
      -- Undecoded
      handler = function(...) return self:write_raw(...) end
    else
      -- Decoded, and the _latest_ timestamp of all objects
      handler = function(...) return self:write(...) end
    end
  elseif type(self) == 'function' then
    -- Grab the entries hashtable
    handler = self
  else
    handler = coyield
  end

  -- Synchronized playback
  local it_log = play_many(logs, {use_iterator=true})
  local t_us_last = 0
  for str, ch, t_us, cnt, idx, ilog in it_log do
    -- true/false implies the channel is desired.
    -- nil implies not desired
    if messages[ch] ~= nil then
      -- Update this entry
      messages[ch] = {str, ch, t_us, cnt, idx, ilog}
      -- Check the messages
      local has_all_entries = true
      for _, v in pairs(messages) do
        if v==false then has_all_entries = false; break end
      end
      -- Check the dt
      local dt_us_last = tonumber(t_us - t_us_last)
      -- We need a new sample when:
      -- There is no sampling rate -OR-
      -- The sampling index channel occurs -OR-
      -- The sampling rate has passed
      local need_new_sample = not dt_us_sample
      if tp_sample == 'string' then
        need_new_sample = dt_us_sample == ch
      elseif tp_sample == 'number' then
        need_new_sample = dt_us_last >= dt_us_sample
      end
      -- Write out our logs
      if has_all_entries and need_new_sample then
        -- print("dt_us_last", dt_us_last / 1e6)
        -- Sorted requires resetting
        if use_log_fmt then
          -- Yield this, in order
          local entries = {}
          for _, message in pairs(messages) do tinsert(entries, message) end
          tsort(entries, by_ts)
          for _, entry in ipairs(entries) do
            -- Handle the message as a log, callback or coroutine
            -- Ignore setting the count, since it doesn't make sense
            local str_entry, ch_entry, t_us_entry = unpack(entry, 1, 3)
            handler(str_entry, ch_entry, t_us_entry)
            -- Clear the message
            messages[ch_entry] = false
          end
        else
          -- Give the decoded payload of {ch: obj}
          local record = {}
          for _, included_ch in ipairs(included_channels) do
            record[included_ch] = decode(messages[included_ch][1])
          end
          handler(record, nil, t_us)
        end
        t_us_last = t_us
      end
    end
  end
  return true
end

return lib
