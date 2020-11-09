-- UNIX FFI library
-- Inspiration: https://github.com/UPenn-RoboCup/UPennDev2/blob/master/Modules/unix/ffi.lua
-- NOTE: We are not thread-safe...

local ffi = require'ffi'

local lib = {}
local C = ffi.C
local function C_has(f_name)
  return pcall(getmetatable(C).__index, C, f_name)
end

local O_RDONLY = 0x0000 -- open for reading only
local O_WRONLY = 0x0001 -- open for writing only
local O_RDWR = 0x0002 -- open for reading and writing
local O_ACCMODE = 0x0003 -- mask for above modes
local O_NONBLOCK = 0x0004 -- no delay
local O_APPEND = 0x0008 -- set append mode
local O_CREAT = 0x0200 -- create if nonexistant
local O_NOCTTY = 0x20000 -- don't assign controlling terminal
local F_GETFL = 3 -- get file status flags
local F_SETFL = 4 -- set file status flags
local F_DUPFD = 0 -- duplicate file descriptor
local O_NDELAY = O_NONBLOCK -- compat

lib.O_RDONLY = O_RDONLY
lib.O_WRONLY = O_WRONLY
lib.O_RDWR = O_RDWR
lib.O_ACCMODE = O_ACCMODE
lib.O_NONBLOCK = O_NONBLOCK
lib.O_APPEND = O_APPEND
lib.O_CREAT = O_CREAT
lib.O_NOCTTY = O_NOCTTY
lib.F_GETFL = F_GETFL
lib.F_SETFL = F_SETFL
lib.F_DUPFD = F_DUPFD

ffi.cdef[[
typedef uint16_t mode_t;
int open(const char *path, int oflag, ...);
int close(int fildes);
long read(int fildes, void *buf, size_t nbyte);
long write(int fildes, const void *buf, size_t nbyte);
char * strerror(int errnum);
void perror(const char *s);
]]

ffi.cdef[[
typedef struct timeval {
  long tv_sec;
  int32_t tv_usec;
} timeval;

typedef long time_t;
typedef struct timespec {
time_t   tv_sec;        /* seconds */
long     tv_nsec;       /* nanoseconds */
} timespec;

int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
int nanosleep(const struct timespec *rqtp, struct timespec *rmtp);
int usleep(uint32_t useconds);
unsigned int sleep(unsigned int seconds);
]]

ffi.cdef[[
typedef struct utsname {
  char sysname[256];  /* [XSI] Name of OS */
  char nodename[256]; /* [XSI] Name of this network node */
  char release[256];  /* [XSI] Release level */
  char version[256];  /* [XSI] Version level */
  char machine[256];  /* [XSI] Hardware type */
} utsname;
int uname(struct utsname *name);

char * realpath(const char * file_name, char * resolved_name);
int chdir(const char *path);
char * getcwd(char *buf, size_t size);
void free(void *ptr);

int gethostname(char *name, size_t namelen);
int mkfifo(const char *path, mode_t mode);
]]

-- Work with file descriptors
ffi.cdef [[
  typedef struct __IO_FILE FILE;
  size_t fwrite
  (const void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
  size_t fread
  (void *restrict ptr, size_t size, size_t nitems, FILE *restrict stream);
  int fileno(FILE *stream);
]]
function lib.fwrite(fp, ptr, sz)
  return C.fwrite(ptr, 1, sz, fp)
end
function lib.fread(fp, ptr, sz)
  return C.fread(ptr, 1, sz, fp)
end

local POLLIN = 0x0001
if not C_has"poll" then
  ffi.cdef[[
  struct pollfd {
      int   fd;      /* file descriptor */
      short events;  /* requested events */
      short revents; /* returned events */
  };
  int poll(struct pollfd *fds, unsigned long int nfds, int timeout);
  ]]
end

ffi.cdef[[
typedef struct fd_set {
  int32_t fds_bits[32];
} fd_set;
int select(int nfds,
           fd_set *restrict readfds,
           fd_set *restrict writefds,
           fd_set *restrict errorfds,
           struct timeval *restrict timeout);
]]

-- Sleep in microseconds
function lib.usleep(useconds)
  return type(useconds)=='number' and useconds > 0 and C.usleep(useconds)
end

function lib.sleep(seconds)
  return type(seconds)=='number' and seconds>0 and C.sleep(seconds)
end

-- NOTE: This is not thread safe!
local dts = ffi.new'timespec'
local dts_remain = ffi.new'timespec'
function lib.nanosleep(t_ns)
  local ns = t_ns % 1e9
  dts.tv_sec = (t_ns - ns) / 1e9
  dts.tv_nsec = ns
  local ret = C.nanosleep(dts, dts_remain)
  if ret==0 then return true end
  return dts_remain.tv_nsec + 1e9 * dts_remain.tv_sec
end

-- Grab the time in seconds
-- Allow time functions to run conversions from a different t
function lib.time(t1)
  local t = t1 or ffi.new'timeval'
  C.gettimeofday(t, nil)
  return tonumber(t.tv_sec) + 1e-6 * t.tv_usec, t
end
function lib.time_ms(t1)
  local t = t1 or ffi.new'timeval'
  C.gettimeofday(t, nil)
  return tonumber(1e3 * t.tv_sec) + 1e-3 * t.tv_usec, t
end
function lib.time_us(t1)
  local t = t1 or ffi.new'timeval'
  C.gettimeofday(t, nil)
  return 1e6 * ffi.cast('uint64_t', t.tv_sec) + t.tv_usec
end

local function errormsg(msg)
  local str = ffi.string(C.strerror(ffi.errno()))
  if type(msg)=='string' then
    return msg..": "..str
  end
  return str
end

function lib.uname()
  local un = ffi.new"utsname"
  if C.uname(un)==0 then
    return ffi.string(un.sysname)
  else
    return false, errormsg()
  end
end

function lib.gethostname()
  local mlen = 128
  local hn = ffi.new("char[?]", mlen)
  if C.gethostname(hn, mlen)==0 then
    return ffi.string(hn)
  else
    return false, errormsg()
  end
end

function lib.open(path, flags)
  if type(path)~='string' then
    return false, "Bad path"
  end
  local fd = C.open(path, flags or O_RDONLY)
  if fd > 0 then return fd end
  return false, errormsg()
end

function lib.fileno(f)
  return type(f)=="userdata" and C.fileno(f)
end

function lib.close(fd)
  return C.close(fd)
end

function lib.read(fd, nbyte)
  local read_buf_sz = nbyte or 1024
  local read_buf = ffi.new("char[?]", read_buf_sz)
  local ret = C.read(fd, read_buf, read_buf_sz)
  if ret > 0 then
    return ffi.string(read_buf, ret)
  elseif ret < 0 then
    return ret
  end
end

function lib.write(fd, item)
  local buf = tostring(item)
  return C.write(fd, buf, #buf)
end

function lib.realpath(dir)
  if type(dir)~='string' then
    return false
  end
  local raw_path = C.realpath(dir, nil)
  local path = ffi.string(raw_path)
  C.free(raw_path)
  return path
end

function lib.getcwd()
  local raw_path = C.getcwd(nil, 0)
  local path = ffi.string(raw_path)
  C.free(raw_path)
  return path
end

function lib.chdir(dir)
  return type(dir)=='string' and C.chdir(dir)
end

-- TODO: Add dirent struct
function lib.readdir()
  local entries = {}
  for entry in io.popen'ls -f1':lines() do
    table.insert(entries, entry)
  end
  return entries
end

function lib.mkfifo(fname, mode)
  if type(fname)~='string' or type(mode)~='number' then
    return false, "Bad argument"
  end
  return C.mkfifo(fname, mode)
end

-- Use the lua built-in
lib.system = os.execute

-- Timeout measured in milliseconds
function lib.poll(_fds, timeout)
  local nfds = #_fds
  local fds = ffi.new('struct pollfd[?]', nfds)
  for i, fd in ipairs(_fds) do
    fds[i-1].fd = fd
    fds[i-1].events = POLLIN
  end
  if type(timeout)~='number' then timeout = -1 end
  local rc = C.poll(fds, nfds, timeout)
  if rc < 0 then
    return false, errormsg()
  elseif rc == 0 then
    return 0
  end
  local events = {}
  for i=1, nfds do
    -- Store the Lua table index
    if fds[i-1].revents == POLLIN then
      table.insert(events, i)
    end
    -- Just gives the state of the FD... maybe ore cumbersome
    -- events[i+1] = (fds[i].revents ~= 0) and fds[i].revents
  end
  return rc, events
end

local bit    = require'bit'
local lshift = bit.lshift
local band   = bit.band
local bor    = bit.bor
local floor  = math.floor
-- Timeout measured in seconds
function lib.select(fds, timeout)
  -- Zero the struct
  local fdset = ffi.new'fd_set'
  -- local nfds = #fds
  local maxfd = 0
  -- Setup the file descriptor set
  for _, fd in ipairs(fds) do
    maxfd = fd > maxfd and fd or maxfd
    --FD_SET(fd, fds)
    fdset.fds_bits[fd / 32] = bor(fdset.fds_bits[fd / 32], lshift(1, fd % 32))
  end
  -- Add a timeout and run the syscall
  local status
  if timeout then
    local to = ffi.new'timeval'
    local integral = floor(timeout)
    to.tv_sec = integral
    to.tv_usec = (timeout - integral) * 1E6
    status = C.select(maxfd + 1, fdset, nil, nil, to)
  else
    status = C.select(maxfd + 1, fdset, nil, nil, nil)
  end
  -- Parse the status
  local set = {}
  for _, fd in ipairs(fds) do
    --table.insert(set, C.FD_ISSET(fd, fds)~=0)
    --table.insert(set, band(fdset.fds_bits[fd / 32], lshift(1, fd % 32)) ~= 0)
    set[fd] = band(fdset.fds_bits[fd / 32], lshift(1, fd % 32)) ~= 0
  end
  return status, set
end
local pread = false
lib.pread = pread
--posix_fadvise
--[[
local mt = {}
-- Provide raw access to all of our functions and variables
-- TODO: Should really just save the constants ourselves...
mt.__index = function(t, k)
  return C[k]
end
return setmetatable(unix, mt)
--]]
return lib
