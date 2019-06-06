#!/usr/bin/env luajit
local fname_lmp = assert(arg[1], "No log specified")
local unpack = unpack or require'table'.unpack
local stringify = require'cjson'.encode
local logger = require'logger'

-- Check if the file exists
local fname_osm = fname_lmp:gsub("%.lmp$", ".osm")
local fname_segments = fname_lmp:gsub("%.lmp$", ".segments.json")
local fname_links = fname_lmp:gsub("%.lmp$", ".links.json")
local fname_bounds = fname_lmp:gsub("%.lmp$", ".bounds.json")
local fname_map = fname_lmp:gsub("%.lmp$", ".map.pgm")
print("Writing map", fname_map)

local function exists(fname)
  local f = io.open(fname, "r")
  if not f then return false, "Does not exist" end
  f:close()
  return true
end

-- Make sure that the input file exists
assert(exists(fname_lmp))

local function get_log_bounds(fname_lmp)
  -- Find the bounds
  local minlat, minlon = math.huge, math.huge
  local maxlat, maxlon = -math.huge, -math.huge
  local options = {
    use_iterator = true,
  }
  local log = logger.open(fname_lmp)
  local it_log, sz_log = log:play(options)
  for str, _, t_us, count in it_log do
    -- Show the progress
    -- print(count, t_us)
    local obj = logger.decode(str)
    -- print(stringify(obj['POSLVData']['position_lla']))
    local lat, lon = unpack(obj['POSLVData']['position_lla'])
    minlat = math.min(minlat, lat)
    minlon = math.min(minlon, lon)
    maxlat = math.max(maxlat, lat)
    maxlon = math.max(maxlon, lon)
  end
  return {minlat, minlon}, {maxlat, maxlon}
end

-- Grab the OSM file if need be
print("Checking OSM", fname_osm)
if not exists(fname_osm) then
  print("Grabbing the OSM file for this log")
  local lla_min, lla_max = get_log_bounds(fname_lmp)
  local proj = require'proj'
  local url_overpass = proj.url_from_minmax(lla_min, lla_max)
  local cmd = string.format('wget "%s" -O %s', url_overpass, fname_osm)
  os.execute(cmd)
end

if not exists(fname_segments) then
  -- local roads = require'roads'
  -- roads.process(fname_osm)
  os.execute("roads.lua "..fname_osm)
end

-- Find the roadways
local has_cjson, cjson = pcall(require, 'cjson')
print("has_cjson", has_cjson)
local bounds = cjson.decode(io.open(fname_bounds):read"*all")
print("bounds", bounds)
local segments = cjson.decode(io.open(fname_segments):read"*all")
print("segments", segments)

local nSegments = bounds.nSegments

local proj = require'proj'
local pr = proj.new{(bounds.minlat + bounds.maxlat) / 2, (bounds.minlon + bounds.maxlon) / 2}

local xmin, ymin = pr:enu(bounds.minlat, bounds.minlon)
local xmax, ymax = pr:enu(bounds.maxlat, bounds.maxlon)

local grid = require'grid'
local my_grid = assert(grid.new{
  scale = 1,
  xmin = xmin, xmax = xmax,
  ymin = ymin, ymax = ymax
})

-- print(segments)
local cnt = 0
for _, v in pairs(segments) do
  cnt = cnt + 1
  print(string.format("Drawing segment %d of %d", cnt, nSegments))
  local pts = v.points
  for i=2,#pts do
    -- Convert
    local ptA = {pr:enu(unpack(pts[i-1]))}
    local ptB = {pr:enu(unpack(pts[i]))}
    print("PtA", unpack(ptA))
    print("PtB", unpack(ptB))
    my_grid:bresenham(ptA, ptB)
  end
end
my_grid:save(fname_map)