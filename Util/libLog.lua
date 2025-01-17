local libLog = {}
local LOG_DIR = '/tmp'
local carray
local ok, ffi = pcall(require, 'ffi')
local C
if ok then
	C = ffi.C
	ffi.cdef [[
	typedef struct __IO_FILE FILE;
	size_t fwrite
	(const void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
	size_t fread
	(void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
	]]
else
	-- TODO: Maybe use a pcall here in case carray not found
	carray = require'carray'
end
local mpack = require'msgpack.MessagePack'.pack

-- TODO: __gc should call stop
local function stop(self)
	-- Close the files
	self.f_meta:close()
	if self.f_raw then self.f_raw:close() end
end

-- For proper recording
local function record(self, meta, raw, n_raw)
	-- Record the metadata
	local mtype, m_ok = type(meta), false
	if mtype=='string' then
		m_ok = self.f_meta:write(meta)
	elseif mtype then
		m_ok = self.f_meta:write(mpack(meta))
	end
	-- Record the raw
	local rtype, r_ok = type(raw), false
	if rtype=='userdata' or rtype=='cdata' then
		-- If no FFI, then cannot record userdata
		-- If no number of raw data, then cannot record
		-- TODO: Use carray as FFI fallback
		if not n_raw then return end
		if C then
			local n_written = C.fwrite(raw, 1, n_raw, self.f_raw)
			r_ok = n_written==n_raw
		else
			r_ok = self.f_raw:write(tostring(carray.byte(raw, n_raw)))
		end
	elseif rtype=='string' then
		r_ok = self.f_raw:write(raw)
	end
	-- Return the status of the writes
  self.n = self.n + 1
	return m_ok, r_ok
end


-- Factory
function libLog.new(prefix, has_raw, dir)
	dir = dir or LOG_DIR
	-- Set up log file handles
  local filetime = os.date('%m.%d.%Y.%H.%M.%S')
  local meta_filename = string.format('%s/%s_m_%s.log',dir,prefix,filetime)
	local raw_filename  = string.format('%s/%s_r_%s.log',dir,prefix,filetime)
	local f_meta = io.open(meta_filename,'w')
	local f_raw, f_raw_c
	if has_raw then f_raw = io.open(raw_filename,'w') end
	-- Set up the object
	local t = {}
	t.f_raw = f_raw
	t.f_meta = f_meta
	t.record = record
	t.stop = stop
  t.n = 0
	return t
end

local function unroll_meta(self)
	-- Read the metadata
	local f_m = assert(io.open(self.m_name,'r'))
	-- Need the C version for unpacker
	local munpacker = require'msgpack'.unpacker
	local metadata = {}
	local u = munpacker(2048)
	local buf, nbuf = f_m:read(512),0
	while buf do
		nbuf = nbuf + #buf
		local res,left = u:feed(buf)
		local tbl = u:pull()
		while tbl do
			metadata[#metadata+1] = tbl
			tbl = u:pull()
		end
		buf = f_m:read(left)
	end
	f_m:close()
  self.metadata = metadata
	return metadata
end

local function unroll_meta2(self)
	-- Read the metadata
	local f_m = assert(io.open(self.m_name,'r'))
	local m = f_m:read('*all')
	f_m:close()
	local unpacker = require'msgpack.MessagePack'.unpacker
	local it = unpacker(m)
	local metadata = {}
	for i, v in it do
		table.insert(metadata, v)
		--print(i, v)
	end
	self.metadata = metadata
	return metadata
end


local function log_iter(self)
	local metadata, buf_t = self.metadata
	local f_r = io.open(self.r_name, 'r')
	local i, n = 0, #metadata
	local function iter(param, state)
		i = i + 1
		if i > n then
			if f_r then f_r:close() end
			return nil
		end
		--if not param then return end
		local m = metadata[i]
		assert(type(m)=='table', 'log_iter | No metadata!')
		-- Metadata only
		if not f_r then return i, m end
		assert(type(m.rsz)=='number', 'log_iter | No raw size given!')
		if C then
			local buf_t = ffi.new('uint8_t[?]', m.rsz)
			local n_read = C.fread(buf_t, 1, m.rsz, f_r)
			return i, m, ffi.string(buf_t, ffi.sizeof(buf_t))
		else
			local data = f_r:read(m.rsz)
			return i, m, data
		end
	end
	return iter, n
end

function libLog.open(dir,date,prefix)
	local t = {}
	t.m_name = dir..'/'..prefix..'_m_'..date..'.log'
	t.r_name = dir..'/'..prefix..'_r_'..date..'.log'
	t.unroll_meta = unroll_meta
	t.unroll_meta2 = unroll_meta2
	t.log_iter = log_iter
	return t
end

-- Take in a table
function libLog.one(filename, meta, raw, sz)

	-- Record the metadata
	local f_meta = io.open('/tmp/'..filename..'_m.log', 'w')
	f_meta:write(mpack(meta))
	f_meta:close()

	if not raw then return end
	local f_raw = io.open('/tmp/'..filename..'_r.log','w')

	-- Record the raw
	local rtype = type(raw)
	if rtype=='userdata' or rtype=='cdata' then
		-- If no FFI, then cannot record userdata
		-- If no number of raw data, then cannot record
		-- TODO: Use carray as FFI fallback
		assert(type(sz)=='number', 'No raw size')
		if C then
			local n_written = C.fwrite(raw, 1, sz, f_raw)
			assert(n_written==sz, 'Bad write size')
		else
			f_raw:write(tostring(carray.byte(raw, sz)))
		end
	elseif rtype=='string' then
		f_raw:write(raw)
	end

	f_raw:close()
end

-- Fill the metatable

return libLog
