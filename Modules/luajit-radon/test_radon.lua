#!/usr/bin/env luajit
local ffi = require'ffi'
local w, h = 40, 30
local edges = ffi.new("uint8_t[?]", w * h)

local function get_r(i, j)
  return math.sqrt(math.pow(i,2)+math.pow(j,2))
end

for j=0,h-1 do
  for i=0,w-1 do
    local ind = j * w + i
    edges[ind] = j==10 and 1 or edges[ind]
    edges[ind] = i==10 and 1 or edges[ind]
    edges[ind] = i==j and 1 or edges[ind]
    edges[ind] = (i+j==50) and 1 or edges[ind]
  end
end

for j=0,h-1 do
  local tbl = {}
  for i=0,w-1 do
    local ind = j * w + i
    table.insert(tbl, edges[ind])
  end
  print(string.format("%2d: %s", j, table.concat(tbl, ' ')))
end

local radon = require'radon'
local maxr = get_r(w, h)
local scaling = 2
local filter = radon.init(maxr, scaling)
-- print(filter)

filter:lines_from_edges(edges, w, h)
-- print(props)

print()
print("Peaks should be at (th, rho)")
print(string.format("Diag %d, %.2f", 135, 10))
print(string.format("Horiz %d, %.2f", 90, 10))
print(string.format("Vert %d, %.2f", 0, 10))
print(string.format("Diag %d, %.2f",
                    45, filter.RSCALE*math.floor(get_r(25/filter.RSCALE, 25/filter.RSCALE))))
print()

local rs = {}
local hline = {"---------"}
for ir=0,filter.MAXR-1 do
  table.insert(rs, string.format("%2d", ir*filter.RSCALE))
  table.insert(hline, "--")
end
print(string.format("rs:     | %s",
                    table.concat(rs, ' ')))
print(table.concat(hline, '-'))

for ith=0,filter.NTH-1 do
  local tbl = {}
  for ir=0,filter.MAXR-1 do
    table.insert(tbl, string.format("%2d",
                                    filter.count_d[ith][ir]))
  end
  print(string.format("th: %3d | %s",
                      ith*filter.ITH_TO_TH, table.concat(tbl, ' ')))
end
print("Max Count", filter.countMax)

local a,b,c = filter:get_line_segment()
print("Min",unpack(a))
print("Max",unpack(b))
print("Mean",unpack(c))

local fname = arg[1]
if not fname then os.exit() end
local has_unix, unix = pcall(require, 'unix')
if not has_unix then os.exit() end
local function load_pgm(fname)
  if type(fname)~='string' then
    return false, "Bad filename"
  end
  local f_pgm
  if fname:match"%.pgm$" then
    f_pgm = io.open(fname)
  elseif fname:match"%.pgm.gz$" then
    f_pgm = io.popen("gzip -dc "..fname)
  end
  if not f_pgm then return false end

  local state = "magic"
  local comments = {}
  local width, height
  repeat
    local line = f_pgm:read"*line"
    if line:match("^#") then
      table.insert(comments, line)
    elseif state=='magic' then
      if line ~="P5" then return false, "Bad magic" end
      state = 'dims'
    elseif state=='dims' then
      width, height = line:match"(%d+) (%d+)"
      if width and height then
        width = tonumber(width)
        height = tonumber(height)
      elseif not width then
        width = line:match"(%d+)"
        width = tonumber(width)
      elseif not height then
        height = line:match"(%d+)"
        height = tonumber(height)
      end
      state = 'maxval'
    elseif state=='maxval' then
      local maxval = tonumber(line)
      if maxval~=255 then return false, "Bad maxval" end
      state = 'data'
    end
  until state == "data"
  local n_cells = width * height
  local map = ffi.new("uint8_t[?]", n_cells)
  unix.fread(f_pgm, map, n_cells)
  f_pgm:close()
  return map, width, height
end
local image, w, h = assert(load_pgm(fname))
local maxr = get_r(w, h)
local scaling = 4
local filter = radon.init(maxr, scaling)
filter:lines_from_image(image, w, h, 40)

print()

for j=0,h-1 do
  local tbl = {}
  for i=0,w-1 do
    local ind = j * w + i
    table.insert(tbl, string.format("%02x", image[ind]))
  end
  -- print(string.format("%2d: %s", j, table.concat(tbl, ' ')))
  print(table.concat(tbl, ' '))
end

print()

local rs = {}
local hline = {"---------"}
for ir=0,filter.MAXR-1 do
  table.insert(rs, string.format("%3d", ir*filter.RSCALE))
  table.insert(hline, "--")
end
print(string.format("rs:     | %s",
                    table.concat(rs, ' ')))
print(table.concat(hline, '-'))

for ith=0,filter.NTH-1 do
  local tbl = {}
  for ir=0,filter.MAXR-1 do
    table.insert(tbl, string.format("%3d",
                                    filter.count_d[ith][ir]))
  end
  print(string.format("th: %3d | %s",
                      ith*filter.ITH_TO_TH, table.concat(tbl, ' ')))
end
print("Max Count", filter.countMax)

local a,b,c = filter:get_line_segment()
print("Min",unpack(a))
print("Max",unpack(b))
print("Mean",unpack(c))