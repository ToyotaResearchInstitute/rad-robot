#!/usr/bin/env luajit

local c_prog_tbl = {[[
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
]],
[[
int main(int argc, char *argv[])
{
]],}

-- Insert the constants we wish to know
local c_constants = {
  AF_INET=true,
  AF_UNIX = true,
  INADDR_ANY=true,
  SOCK_DGRAM=true,
  SOCK_STREAM = true,
  SOCK_SEQPACKET = true,
  SOL_SOCKET=true,
  SO_REUSEADDR=true,
  SO_REUSEPORT = true,
  IPPROTO_IP = true,
  IP_ADD_MEMBERSHIP = true,
  SOCK_DGRAM = true,
  IP_MULTICAST_TTL = true,
  IP_MULTICAST_LOOP = true,
  SO_RCVBUF = true,
  SO_SNDBUF = true,
  SO_TIMESTAMP = true,
  MSG_DONTWAIT = true,
  SO_BROADCAST = true
}

-- Alphabetical
local keys = {}
for k in pairs(c_constants) do
  table.insert(keys, k)
end
table.sort(keys)

local constants_fmt='\tfprintf(stdout, "local %s = 0x%%02X -- %%lu bytes\\n", %s, sizeof(%s));'
for _, k in ipairs(keys) do
  table.insert(c_prog_tbl,
    string.format(constants_fmt, k, k, k))
end
table.insert(c_prog_tbl,"}")

-- Generate the file string through concatenation
local c_prog_str = table.concat(c_prog_tbl,'\n')
if DEBUG then
  io.stderr:write("\t= Debug Program =\n")
  io.stderr:write(c_prog_str, '\n')
end

-- Write the file to a tmp file for compiling
local fname = os.tmpname()
local f_prog = io.open(fname..".c", "w")
f_prog:write(c_prog_str)
f_prog:close()

-- Compile the generated C program
io.stderr:write("\t==Generating==\n")
local compile_str = "cc -o "..fname.." "..fname..".c"
local ret = os.execute(compile_str)

assert(os.remove(fname..".c"))

io.stderr:write("\t==Grabbing the constants==\n")
local f_gen = io.popen(fname)
local constants = f_gen:read('*all')
io.stderr:write("\t==Results==\n")
print(constants)

assert(os.remove(fname))
