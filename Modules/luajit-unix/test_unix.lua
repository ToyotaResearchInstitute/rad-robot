#!/usr/bin/env luajit
local unix = require'unix'

print("\n ** LIBRARY **")
for k,v in pairs(unix) do
  print(k, type(v))
end

print("\n ** TIME TEST **")
local t0 = unix.time()
print(t0)

print("\n ** UNAME TEST **")
print(unix.uname())

print("\n ** HOSTNAME TEST **")
print(unix.gethostname())

print("\n ** GETCWD TEST **")
local path = unix.getcwd()
print(path)

print("\n ** OPEN TEST **")
local fd = unix.open("README.md", unix.O_RDONLY)
print(fd)

print("\n ** READ TEST **")
local ret = unix.read(fd)
print(ret)

print("\n ** CLOSE TEST **")
print(unix.close(fd))

print("\n ** READDIR TEST **")
local dir = unix.readdir(path)
for k, v in pairs(dir) do
  print(k, v)
end

print("\n ** SELECT TEST **")
local status, available = unix.select({}, 1)
print(status, available)
