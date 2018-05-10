-- AUTHOR: Stephen McGill, 2017
-- Usage: luajit test_pcap.lua FILENAME.pcap
-- Description: Parse PCAP files from tcpdump

local lib = {}

---------------
-- Dependencies
local ffi = require'ffi'
local mmap = require'mmap'
local unpack = unpack or require'table'.unpack
---------------

------------------
-- PCAP file format: http://wiki.wireshark.org/Development/LibpcapFileFormat/
ffi.cdef[[
typedef struct pcap_file {
  /* file header */
  uint32_t magic_number;   /* magic number */
  uint16_t version_major;  /* major version number */
  uint16_t version_minor;  /* minor version number */
  int32_t  thiszone;       /* GMT to local correction */
  uint32_t sigfigs;        /* accuracy of timestamps */
  uint32_t snaplen;        /* max length of captured packets, in octets */
  uint32_t network;        /* data link type */
} __attribute__((packed)) pcap_file;

/* This is the header of a packet on disk.  */
typedef struct pcap_record {
  /* record header */
  uint32_t ts_sec;         /* timestamp seconds */
  uint32_t ts_usec;        /* timestamp microseconds */
  uint32_t incl_len;       /* number of octets of packet saved in file */
  uint32_t orig_len;       /* actual length of packet */
} __attribute__((packed)) pcap_record;
]]
------------------
local pcap_hdr_sz = ffi.sizeof'pcap_file'
local record_hdr_sz = ffi.sizeof'struct pcap_record'
-- Export
-- lib.pcap_hdr_sz = pcap_hdr_sz
-- lib.record_hdr_sz = record_hdr_sz

--------------------------
-- Iterator on the entries
-- Iterator yield timestamp, packet w/ length, record w/length
function lib.entries(fname)
  if type(fname) ~='string' then
    return false, "Bad filename"
  end
  local mobj = mmap.open(fname)
  local ptr, sz = unpack(mobj)
  local pcap_hdr = ffi.cast("pcap_file *", ptr)
  if pcap_hdr.magic_number == 0xA0D0D0A then
    return false, string.format('%s: Bad magic number: %X. Convert from pcap-ng to pcap',
                                fname, pcap_hdr.magic_number)
  elseif pcap_hdr.magic_number ~= 0xA1B2C3D4 then
    return false, string.format('%s: Bad magic number: %X', fname, pcap_hdr.magic_number)
  end
  local ptr_end = ptr + sz
  -- local npkt = (sz - ffi.sizeof'pcap_file') / ffi.sizeof('pcap_record')
  ptr = ptr + pcap_hdr_sz
  return coroutine.wrap(function()
    local cnt = 0
    while ptr < ptr_end do
      cnt = cnt + 1
      local ptr_record = ffi.cast('pcap_record*', ptr)
      local t = ptr_record.ts_sec + ptr_record.ts_usec / 1e6
      local incl_len = ptr_record.incl_len
      -- local orig_len = ptr_record.orig_len
      local packet = ptr + record_hdr_sz
      -- local packet = ffi.cast("uint8_t*", ptr_record + 1)
      coroutine.yield(t, {packet, incl_len}, {ptr_record, record_hdr_sz + incl_len})
      ptr = packet + incl_len
    end
    mmap.close(mobj)
    return
  end), {pcap_hdr, pcap_hdr_sz}
end

return lib
