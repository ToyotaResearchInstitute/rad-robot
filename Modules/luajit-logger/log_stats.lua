#!/usr/bin/env luajit
local logger = require'logger'
local fname = assert(arg[1], "No log specified")
assert(fname:match(logger.iso8601_match), "Bad log datestamp")

local channels = {}
local sizes = {}
local t_first = {}
local t_last = {}
local max_wait = {}
local min_wait = {}

local t_us0, t_us1
for str, ch, t_us, _ in logger.play(fname, true) do
  if not t_us0 then t_us0 = t_us end
  t_us1 = t_us
  channels[ch] = (channels[ch] or 0) + 1
  sizes[ch] = (sizes[ch] or 0) + #str
  -- Timing
  if t_last[ch] then
    local dt = tonumber(t_us - t_last[ch])
    max_wait[ch] = math.max(dt, max_wait[ch])
    min_wait[ch] = math.min(dt, min_wait[ch])
  else
    t_last[ch] = t_us
    max_wait[ch] = -math.huge
    min_wait[ch] = math.huge
  end
  if not t_first[ch] then t_first[ch] = t_us end
  t_last[ch] = t_us
end

local info = {
  string.format("Log spanned %.2f seconds",
    tonumber(t_us1 - t_us0) / 1e6),
  table.concat({
    "Channel","Count","Frequency (Hz)","Avg (ms)","Min (ms)","Max (ms)","Size (bytes)"
    },'\t')
}
for ch, n in pairs(channels) do
  local duration = tonumber(t_last[ch] - t_first[ch])
  local frequency = n / duration * 1e6
  local period = duration / n / 1e3 -- milliseconds
  table.insert(info, string.format("%s\t%d\t%.1f\t%.3f\t%.3f\t%.3f\t%.1f",
    ch, n, frequency, period, min_wait[ch]/1e3, max_wait[ch]/1e3, sizes[ch]/n))
end
io.write(table.concat(info, '\n'), '\n')

