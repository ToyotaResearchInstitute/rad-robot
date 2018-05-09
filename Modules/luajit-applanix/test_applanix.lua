#!/usr/bin/env luajit

local ffi = require'ffi'
local C = ffi.C
local a = require'applanix'

ffi.cdef [[
typedef struct timeval {
  long tv_sec;
  int32_t tv_usec;
} timeval;
int gettimeofday(struct timeval *restrict tp, void *restrict tzp);
]]
local t = ffi.new'timeval'
local function utime()
  C.gettimeofday(t, nil)
  return 1e6 * t.tv_sec + t.tv_usec
end

local update = a.init()
while true do
  local ch, data = update()
  local t = utime()
  if ch=="Nav" then
    -- Navigation Message
    print(t, ch, data)
  end
end
