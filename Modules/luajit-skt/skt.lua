local lib = {}

local ffi = require'ffi'
local C = ffi.C

-- local time = require'unix'.time

local function C_has(f_name)
  return pcall(getmetatable(C).__index, C, f_name)
end

-- Multicast
-- OSX requires:
-- sudo route add -net 224.0.0.0 -interface lo0 240.0.0.0
-- Linux:
-- sudo ifconfig lo multicast
-- sudo route add -net 224.0.0.0 netmask 240.0.0.0 dev lo

-- TODO: Inspect CMSG Macros for file descriptor sending and timestamping
-- http://man7.org/linux/man-pages/man3/cmsg.3.html
-- http://man.openbsd.org/CMSG_DATA.3

--------------------
-- From the platform-specific generator
-- This is Linux
local AF_INET = 0x02 -- 4 bytes
local AF_UNIX = 0x01 -- 4 bytes
local IP_ADD_MEMBERSHIP = 0x23 -- 4 bytes
local IP_MULTICAST_LOOP = 0x22
local IP_MULTICAST_TTL = 0x21 -- 4 bytes
local IPPROTO_IP = 0x00 -- 4 bytes
local INADDR_ANY = 0x00 -- 4 bytes
local MSG_DONTWAIT = 0x40 -- 4 bytes
local SO_BROADCAST = 0x06 -- 4 bytes
local SO_SNDBUF = 0x07 -- 4 bytes
local SO_RCVBUF = 0x08 -- 4 bytes
local SO_REUSEADDR = 0x02 -- 4 bytes
local SO_REUSEPORT = 0x0F -- 4 bytes
local SO_TIMESTAMP = 0x1D -- 4 bytes
local SOCK_DGRAM = 0x02 -- 4 bytes
local SOCK_SEQPACKET = 0x05 -- 4 bytes
local SOCK_STREAM = 0x01 -- 4 bytes
local SOL_SOCKET = 0x01 -- 4 bytes
--------------------

if ffi.os=='OSX' then
  AF_INET = 0x02 -- 4 bytes
  AF_UNIX = 0x01 -- 4 bytes
  INADDR_ANY = 0x00 -- 4 bytes
  IPPROTO_IP = 0x00 -- 4 bytes
  IP_ADD_MEMBERSHIP = 0x0C -- 4 bytes
  IP_MULTICAST_LOOP = 0x0B -- 4 bytes
  IP_MULTICAST_TTL = 0x0A -- 4 bytes
  MSG_DONTWAIT = 0x80 -- 4 bytes
  SOCK_DGRAM = 0x02 -- 4 bytes
  SOCK_SEQPACKET = 0x05 -- 4 bytes
  SOCK_STREAM = 0x01 -- 4 bytes
  SOL_SOCKET = 0xFFFF -- 4 bytes
  SO_BROADCAST = 0x20 -- 4 bytes
  SO_SNDBUF = 0x1001 -- 4 bytes
  SO_RCVBUF = 0x1002 -- 4 bytes
  SO_REUSEADDR = 0x04 -- 4 bytes
  SO_REUSEPORT = 0x200 -- 4 bytes
  SO_TIMESTAMP = 0x400 -- 4 bytes
end

local UDP_HEADER_SZ = 20
local BATCH_SZ = 8
local MAX_LENGTH = 65535 -- Jumbo UDP packet
local SKT_BUFFER_SZ = math.pow(2, 20)
local UNIX_PATH_MAX = 108

ffi.cdef[[
struct sockaddr {
  unsigned short sa_family;    // address family, AF_xxx
  char           sa_data[14];  // 14 bytes of protocol address
};
typedef struct in_addr {
  uint32_t s_addr;
} in_addr;

typedef struct sockaddr_un {
  uint16_t sun_family;
  char sun_path[108];
} sockaddr_un;
]]

-- Establish the network structs for each platform
if ffi.os=='Linux' then
  ffi.cdef[[
  typedef struct sockaddr_in {
    uint16_t sin_family;
    uint16_t sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
  } sockaddr_in;
  ]]
else
  ffi.cdef[[
  typedef struct sockaddr_in {
    uint8_t sin_len;
    uint8_t sin_family;
    uint16_t sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
  } sockaddr_in;
  ]]
end

-- TODO: Check socklen_t validity
ffi.cdef[[
typedef int ssize_t;
typedef uint32_t socklen_t;

uint32_t htonl(uint32_t hostlong);
uint16_t htons(uint16_t hostshort);
uint32_t ntohl(uint32_t netlong);
uint16_t ntohs(uint16_t netshort);
int inet_aton(const char *cp, struct in_addr *pin);

int socket(int domain, int type, int protocol);
int fcntl(int fildes, int cmd, ...);
int connect(int socket, const struct sockaddr *address, uint32_t address_len);
int bind(int socket, const struct sockaddr *address, uint32_t address_len);
int setsockopt(int socket, int level, int option_name, const void *option_value, uint32_t option_len);
int getsockopt(int sockfd, int level, int optname, void *optval, uint32_t *optlen);

ssize_t send(int socket, const void *buffer, size_t length, int flags);
ssize_t recv(int socket, void *buffer, size_t length, int flags);
ssize_t sendto(int socket, const void *buffer, size_t length, int flags,
  const struct sockaddr *dest_addr, socklen_t dest_len);
ssize_t recvfrom(int socket, void *restrict buffer, size_t length, int flags,
  struct sockaddr *restrict address, socklen_t *restrict address_len);

typedef struct hostent {
  char    *h_name;        /* official name of host */
  char    **h_aliases;    /* alias list */
  int     h_addrtype;     /* host address type */
  int     h_length;       /* length of address */
  char    **h_addr_list;  /* list of addresses from name server */
} hostent;
struct hostent * gethostbyname(const char *name);
]]

if not C_has"close" then ffi.cdef"int close(int fildes);" end
if not C_has"perror" then ffi.cdef"void perror(const char *s);" end

-- Add multicast support
ffi.cdef[[
typedef struct ip_mreq {
  struct in_addr imr_multiaddr;  /* IP multicast address of group */
  struct in_addr imr_interface;  /* local IP address of interface */
} ip_mreq;
]]

ffi.cdef[[
struct iovec {                    /* Scatter/gather array items */
  void  *iov_base;              /* Starting address */
  size_t iov_len;               /* Number of bytes to transfer */
};
struct msghdr {
  void         *msg_name;       /* optional address */
  socklen_t     msg_namelen;    /* size of address */
  struct iovec *msg_iov;        /* scatter/gather array */
  size_t        msg_iovlen;     /* # elements in msg_iov */
  void         *msg_control;    /* ancillary data, see below */
  size_t        msg_controllen; /* ancillary data buffer len */
  int           msg_flags;      /* flags on received message */
};

struct cmsghdr {
  size_t cmsg_len;   /* data byte count, including hdr */
  int    cmsg_level; /* originating protocol */
  int    cmsg_type;  /* protocol-specific type */
};

ssize_t sendmsg(int socket, const struct msghdr *message, int flags);
ssize_t recvmsg(int sockfd, struct msghdr *msg, int flags);
]]

-- Add poll mechanism
local POLLIN = 0x0001
if not C_has"poll" then
  -- Inspiration: https://github.com/UPenn-RoboCup/UPennDev2/blob/master/Modules/unix/ffi.lua
  ffi.cdef[[
  struct pollfd {
      int   fd;      /* file descriptor */
      short events;  /* requested events */
      short revents; /* returned events */
  };
  int poll(struct pollfd *fds, unsigned long int nfds, int timeout);
  ]]
end

-- timeout is in milliseconds
-- Returns an array of the revents for each fd
function lib.poll(_fds, timeout)
  local nfds = #_fds
  local fds = ffi.new('struct pollfd[?]', nfds)
  for i, fd in ipairs(_fds) do
    fds[i-1].fd = fd
    fds[i-1].events = POLLIN
  end
  local rc = C.poll(fds, nfds, tonumber(timeout) or -1)
  if rc < 0 then
    C.perror'poll'
    return false, "Bad poll"
  elseif rc==0 then
    return 0
  end
  local events = {}
  for i=0, nfds-1 do
    -- events[_fds[i+1]] = (fds[i-1].revents ~= 0) and fds[i-1].revents
    local revents = fds[i].revents
    events[i+1] = (revents ~= 0) and revents
  end
  return rc, events
end

local function close(self)
  if self.fd then
    C.close(self.fd)
    self.fd = nil
  end
end

local function send(self, data, len)
  local sz = len or #data
  local res = C.send(self.fd, data, sz, 0)
  if res < 0 then
    C.perror"send"
    return false, "Cannot perform send"
  end
  return res
end

local function sendto(self, buffer, length)
  if type(buffer)~="string" then
    return false, "Bad buffer"
  end
  length = length or #buffer
  local res = C.sendto(self.fd, buffer, length, 0,
                       ffi.cast("const struct sockaddr*", self.send_addr),
                       self.addr_sz[0])
  if res < 0 then
    C.perror"sendto"
    return false, string.format("Cannot sendto %d bytes", length)
  end
  return res
end

local function send_all(self, msg)
  if type(msg)=='string' then
    return self:send(msg)
  elseif type(msg)~='table' then
    return false, "Need a table of strings"
  end
  local n = 0
  for i, frag in ipairs(msg) do
    local ret, err = self:send(frag)
    if ret then
      n = n + ret
    -- Silently ignore for now
    -- else
    --   return false, string.format("Pkt %d: %s", i, err)
    end
  end
  return n
end

-- https://github.com/UPenn-RoboCup/UPennDev2/blob/master/Modules/udp/ffi.lua
local function recv(self, block)
  local buf_len = tonumber(C.recv(self.fd, self.buffer, ffi.sizeof(self.buffer),
    block and 0 or MSG_DONTWAIT))
  if buf_len < 0 then
    C.perror"recv"
    return false, "recv"
  end
  return ffi.string(self.buffer, buf_len)
  --return self.buffer, buf_len
end

local function recvfrom(self, block)
  local buf_len = C.recvfrom(self.fd, self.buffer, ffi.sizeof(self.buffer),
    block and 0 or MSG_DONTWAIT, self.addr, self.addr_sz)
  if buf_len < 0 then
    -- C.perror"recvfrom"
    return false, "recvfrom"
  end
  -- Get the address and port information
  local address = self.recv_addr.sin_addr.s_addr
  local port = self.recv_addr.sin_port
  return ffi.string(self.buffer, buf_len), address, port
end

-- Just allow for macOS and Linux
ffi.cdef[[
struct mmsghdr {
  struct msghdr msg_hdr;  /* Message header */
  unsigned int  msg_len;  /* Number of received bytes for header */
};
]]

local recvmmsg
if ffi.os=='OSX' or true then
  recvmmsg = function(self, block)
    local msgs = {}
    repeat
      local pkt, addr, port = recvfrom(self, block)
      if pkt then table.insert(msgs, {pkt, addr, port}) end
    until not pkt
    if #msgs == 0 then
      return false, "OSX recvmmsg"
    end
    return msgs
  end
else
  -- Add recvmmsg
  if not pcall(ffi.new, "timespec") then
    ffi.cdef[[
    typedef long time_t;
    typedef struct timespec {
    time_t   tv_sec;        /* seconds */
    long     tv_nsec;       /* nanoseconds */
    } timespec;
    ]]
  end
  ffi.cdef[[
  int recvmmsg(int sockfd, struct mmsghdr *msgvec, unsigned int vlen,
               unsigned int flags, struct timespec *timeout);
  ]]
  -- TODO: Check implementation
  recvmmsg = function(self, block)
    local nr_datagrams = C.recvmmsg(self.fd, self.datagrams, self.BATCH_SZ,
      block and 0 or MSG_DONTWAIT, nil)
    if nr_datagrams < 1 then
      --C.perror"recvmmsg"
      return false, "recvmmsg"
    end
    local msgs = {}
    for i = 0,(nr_datagrams-1) do
      local sockaddr = ffi.cast('sockaddr_in*', self.datagrams[i].msg_hdr.msg_name)
      local port = C.ntohs(sockaddr[0].sin_port)
      local address = C.ntohl(sockaddr[0].sin_addr.s_addr)
      table.insert(msgs, {ffi.string(self.iovecs[i].iov_base, self.datagrams[i].msg_len), address, port})
    end
    self.counter = self.counter + nr_datagrams
    return msgs
  end
end

local mt = {
  __gc = close
}

function lib.open(parameters)

  if type(parameters)~='table' then
    return false, "Please provide parameters"
  end

  local port = parameters.port
  if type(port)~='number' then
    return false, string.format("Bad port [%s]", type(port))
  end

  -- Sending requires an address
  local address = parameters.address
  local is_ipv4, is_broadcast, is_multicast, is_discovery
  local use_connect
  if type(address)=='string' then
    local ip_a, ip_b, ip_c, ip_d = address:match"^(%d+)%.(%d+)%.(%d+)%.(%d+)$"
    ip_a, ip_b, ip_c, ip_d =
      tonumber(ip_a), tonumber(ip_b), tonumber(ip_c), tonumber(ip_d)
    is_ipv4 = ip_d and ip_a<256 and ip_b<256 and ip_c<256 and ip_d<256
    is_multicast = ip_a >= 224 and ip_a <= 239
    is_discovery = ip_a == 240
    is_broadcast = ip_d == 255 and (not is_multicast) and (not is_discovery)
    use_connect = (not is_multicast) and (not is_discovery)
  elseif parameters.unix then
    return false, "Need an address for unix"
  end
  use_connect = use_connect and not (parameters.use_connect == false)

  -- Open the socket
  local fd = C.socket(
    parameters.unix and AF_UNIX or AF_INET,
    parameters.tcp and SOCK_STREAM or SOCK_DGRAM,
    0)
  if fd <= 0 then
    C.perror"socket"
    return false, "Cannot open socket"
  end

  -- Option setting variables
  local ret
  local option = ffi.new'int[1]'
  local option_void = ffi.cast('const void *', option)
  local option_sz = ffi.new'socklen_t[1]'
  option_sz[0] = ffi.sizeof(option)

  -- Timestamping packets
  option[0] = 1
  ret = C.setsockopt(fd, SOL_SOCKET, SO_TIMESTAMP,
    option_void, option_sz[0])
  if ret < 0 then
    C.perror"SO_TIMESTAMP"
    -- C.close(fd)
    --return false, "Cannot add timestamp support"
  end

  -- Ability to broadcast over IPv4
  local use_broadcast = parameters.use_broadcast or is_broadcast
  option[0] = use_broadcast and 1 or 0
  ret = C.setsockopt(fd, SOL_SOCKET, SO_BROADCAST,
    option_void, option_sz[0])
  if ret < 0 then
    C.perror"SO_BROADCAST"
    C.close(fd)
    return false, "Cannot broadcast on socket"
  end

  -- Ability to resuse the address
  option[0] = 1
  ret = C.setsockopt(fd, SOL_SOCKET, SO_REUSEADDR,
    option_void, option_sz[0])
  if ret < 0 then
    C.perror"SO_REUSEADDR"
    C.close(fd)
    return false, "Cannot reuse address"
  end

  -- Ability to reuse the port
  -- This just for multicast...
  if is_multicast then
    option[0] = 1
    ret = C.setsockopt(fd, SOL_SOCKET, SO_REUSEPORT,
      option_void, option_sz[0])
    if ret < 0 then
      C.perror"SO_REUSEPORT"
      C.close(fd)
      return false, "Cannot reuse port"
    end
  end

  -- Get the buffer
  local rcvbuf_sz
  do
    local goption = ffi.new'int[1]'
    local goption_void = ffi.cast('void *', option)
    local goption_sz = ffi.new'socklen_t[1]'
    ret = C.getsockopt(fd, SOL_SOCKET, SO_RCVBUF,
      goption_void, goption_sz)
    if ret < 0 then
      C.perror"SO_RCVBUF"
      return false, "Cannot get current receive buffer"
    end
    rcvbuf_sz = goption[0]
    -- print(string.format("Recv buffer is %d", rcvbuf_sz))
  end
  -- TODO: Set the buffer
  option[0] = tonumber(parameters.max_length) or SKT_BUFFER_SZ
  ret = C.setsockopt(fd, SOL_SOCKET, SO_RCVBUF,
    option_void, ffi.sizeof(option) )
  if ret < 0 then
    C.perror"SO_RCVBUF"
    return false, "Cannot set receive buffer"
  end
  --print(string.format("Receive buffer is %d", i[0]))

  -- Get the buffer
  local sndbuf_sz
  do
    local goption = ffi.new'int[1]'
    local goption_void = ffi.cast('void *', option)
    local goption_sz = ffi.new'socklen_t[1]'
    ret = C.getsockopt(fd, SOL_SOCKET, SO_RCVBUF,
      goption_void, goption_sz)
    if ret < 0 then
      C.perror"SO_SNDBUF"
      return false, "Cannot get current receive buffer"
    end
    sndbuf_sz = goption[0]
    -- print(string.format("Send buffer is %d", sndbuf_sz))
  end

  -- TODO: Set the buffer
  option[0] = tonumber(parameters.max_length) or SKT_BUFFER_SZ
  ret = C.setsockopt(fd, SOL_SOCKET, SO_SNDBUF,
    option_void, ffi.sizeof(option) )
  if ret < 0 then
    C.perror"SO_SNDBUF"
    return false, "Cannot set receive buffer"
  end
  --print(string.format("Receive buffer is %d", i[0]))

  -- Specify the receiving address information
  local recv_addr
  if parameters.unix then
    -- Set the receiving address information
    recv_addr = ffi.new'sockaddr_un'
    ffi.fill(recv_addr, ffi.sizeof('sockaddr_un'))
    recv_addr.sun_family = AF_UNIX
    -- Ensure null characters in the beginnning and end
    ffi.fill(recv_addr.sun_path, UNIX_PATH_MAX);
    -- null character in the beginning means hidden...
    local len = math.min(#address, UNIX_PATH_MAX - 1)
    ffi.copy(recv_addr.sun_path + 1, address, len)
  else
    recv_addr = ffi.new"sockaddr_in"
    ffi.fill(recv_addr, ffi.sizeof'sockaddr_in')
    recv_addr.sin_family = AF_INET
    recv_addr.sin_port = C.htons(port)
    recv_addr.sin_addr.s_addr = C.htonl(INADDR_ANY)
  end

  -- Bind to the port for receiving
  ret = C.bind(fd, ffi.cast('const struct sockaddr *', recv_addr),
    ffi.sizeof(recv_addr))
  if ret < 0 then
    C.perror"bind"
    C.close(fd)
    return false, "Bad bind"
  end

  -- Receive Many
  -- TODO: Check if mmsghdr is available
  local addrs = ffi.new("struct sockaddr[?]", BATCH_SZ)
  local buffers = {}
  for _=1,BATCH_SZ do
    table.insert(buffers, ffi.new('uint8_t[?]', MAX_LENGTH))
  end
  local datagrams = ffi.new('struct mmsghdr[?]', BATCH_SZ)
  local iovecs = ffi.new('struct iovec[?]', BATCH_SZ)
  -- Initial population
  for i = 0, BATCH_SZ-1 do
    local buf = buffers[i+1]
    iovecs[i].iov_base = buf
    iovecs[i].iov_len  = ffi.sizeof(buf)
    datagrams[i].msg_hdr.msg_iov     = iovecs[i]
    datagrams[i].msg_hdr.msg_iovlen  = 1;
    datagrams[i].msg_hdr.msg_name    = addrs + i
    datagrams[i].msg_hdr.msg_namelen = ffi.sizeof(addrs[i])
  end

  local send_addr
  if parameters.unix then
    send_addr = ffi.new"sockaddr_un"
    ffi.fill(send_addr, ffi.sizeof(send_addr))
    -- Set the receiving address information
    send_addr.sun_family = AF_UNIX
    -- Ensure null characters in the beginnning and end
    ffi.fill(send_addr.sun_path, UNIX_PATH_MAX);
    -- null character in the beginning means hidden...
    local len = math.min(#address, UNIX_PATH_MAX - 1)
    ffi.copy(send_addr.sun_path + 1, address, len)
  else
    send_addr = ffi.new"sockaddr_in"
    ffi.fill(send_addr, ffi.sizeof(send_addr))
    send_addr.sin_family = AF_INET
    -- Specify the same port for listening
    send_addr.sin_port = C.htons(port)
      -- Get the IPv4 Address into network byte order
    if is_ipv4 then
      ret = C.inet_aton(address, send_addr.sin_addr)
      if ret <= 0 then
        C.perror"inet_aton"
        C.close(fd)
        return false, "Cannot translate IP address to in_addr"
      end
    elseif type(address) == 'string' then
      -- Get hostname into network byte order
      local hostptr = C.gethostbyname(address)
      if not hostptr then
        C.perror"gethostbyname"
        C.close(fd)
        return false, "Bad gethostbyname"
      end
      ffi.copy(ffi.cast('uint8_t*', send_addr.sin_addr),
        hostptr.h_addr_list[0], hostptr.h_length)
    end
  end

  -- Add a connection address for send, sendto default
  if use_connect then
    -- Connect (Use send, and not sendto)
    ret = C.connect(fd, ffi.cast('struct sockaddr *', send_addr),
      ffi.sizeof'sockaddr_in')
    if ret < 0 then
      C.perror"connect"
      C.close(fd)
      return false, "Cannot connect"
    end
  end

  if is_multicast then
    -- Add ourselves as a multicast member
    local mreq = ffi.new"ip_mreq"
    ffi.fill(mreq, ffi.sizeof"ip_mreq")
    mreq.imr_multiaddr.s_addr = send_addr.sin_addr.s_addr
    mreq.imr_interface.s_addr = recv_addr.sin_addr.s_addr
    ret = C.setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
      mreq, ffi.sizeof(mreq))
    if ret < 0 then
      C.perror"IP_ADD_MEMBERSHIP"
      return false, "Cannot add ourselves to multicast group"
    end

    -- Set multicast time-to-live
    local ttl = parameters.ttl
    option[0] = tonumber(ttl) or 0
    ret = C.setsockopt(fd, IPPROTO_IP, IP_MULTICAST_TTL,
      option, ffi.sizeof(option))
    if ret < 0 then
      C.perror"IP_MULTICAST_TTL"
      return false, "Cannot set multicast time-to-live"
    end

    -- NOTE: Multicast loopback is enabled by default
    --[[
    option[0] = 1
    ret = C.setsockopt(fd, IPPROTO_IP, IP_MULTICAST_LOOP,
      option, ffi.sizeof(option))
    if ret < 0 then
      C.perror"IP_MULTICAST_LOOP"
      return false, "Cannot set multicast loopback"
    end
    --]]
  end

  -- This is just for sendto functionality, which is precluded by using "connect"
  -- send_addr = ffi.cast("const struct sockaddr*", send_addr)
  local addr_sz = ffi.new("socklen_t[1]")
  addr_sz[0] = ffi.sizeof(send_addr)

  return setmetatable({
    address = address,
    port = port,
    fd = fd,
    close = close,
    addr_sz = addr_sz,
    -- Sending
    send_addr = send_addr,
    send = use_connect and send or sendto,
    send_all = send_all,
    -- Receiving
    recv_addr = recv_addr,
    recv = parameters.use_recv and recv or recvfrom,
    recvmmsg = recvmmsg,
    -- Receive Many
    datagrams = datagrams,
    iovecs = iovecs,
    BATCH_SZ = BATCH_SZ,
    buffers = buffers,
    buffer = buffers[1],
    }, mt)
end

return lib
