#!/usr/bin/env luajit

local lib = {}
local sformat = require'string'.format
local unpack = unpack or require'table'.unpack

local ffi = require'ffi'
local proj = ffi.load"proj"
ffi.cdef[[
void* proj_create (void *, char*);
void* proj_destroy(void *);
typedef union { double v[4]; } PJ_COORD;
PJ_COORD proj_trans (void *, int, PJ_COORD);
PJ_COORD proj_geod(void *, PJ_COORD, PJ_COORD);
]]
local PJ_FWD = 1
local PJ_INV = -1

local function enu(self, lat, lon, alt)
  local a = ffi.new("PJ_COORD")
  a.v[0] = math.rad(lon)
  a.v[1] = math.rad(lat)
  a.v[2] = tonumber(alt) or 0
  local b = proj.proj_trans(self.P, PJ_FWD, a)
  local e, n, u = b.v[0], b.v[1], b.v[2]
  return e, n, u
end

local function lla(self, east, north, up)
  local a = ffi.new("PJ_COORD")
  a.v[0] = east
  a.v[1] = north
  a.v[2] = tonumber(up) or 0
  local b = proj.proj_trans(self.P, PJ_INV, a)
  local lon, lat, alt = b.v[0], b.v[1], b.v[2]
  return math.deg(lat), math.deg(lon), alt
end

local function geodesic(self, lla1, lla2)
  local a = ffi.new("PJ_COORD")
  a.v[0] = math.rad(lla1[2])
  a.v[1] = math.rad(lla1[1])
  a.v[2] = tonumber(lla1[3]) or 0
  local b = ffi.new("PJ_COORD")
  b.v[0] = math.rad(lla2[2])
  b.v[1] = math.rad(lla2[1])
  b.v[2] = tonumber(lla2[3]) or 0
  local c = proj.proj_geod(self.P, a, b);
  return c.v[0]
end

local function new(lla0)
  local lat0, lon0 = unpack(lla0, 1, 2)
  local tfm = table.concat({
    "+proj=tmerc", --"+proj=etmerc",
    sformat("+lat_0=%.7f", lat0),
    sformat("+lon_0=%.7f", lon0),
    "+ellps=WGS84",
    "+units=m",
    "+no_defs",
    "+k_0=0.999966667",
    "+x0=0",
    "+y_0=0"
  }, " ")
  tfm = ffi.new("char[?]", #tfm+1, tfm)
  local P = ffi.gc(proj.proj_create(nil, tfm), proj.proj_destroy)
  return {
    P = P,
    lla_center = {lat0, lon0, 0},
    enu = enu,
    lla = lla,
    geodesic = geodesic
  }
end
lib.new = new

local function get_osm(center_lla, radius_meters)
  radius_meters = tonumber(radius_meters) or 1e3
  local pr = new(center_lla)
  local lat_min, lon_min = pr:lla(-radius_meters, -radius_meters)
  local lat_max, lon_max = pr:lla(radius_meters, radius_meters)
  local bbox = sformat("%.7f,%.7f,%.7f,%.7f", lat_min, lon_min, lat_max, lon_max)
  local kinds = sformat("way[lanes][highway!=footway];way[highway][highway!=footway];")
  local query = sformat("[bbox:%s];(%s);(._;>;);out;", kinds, bbox)
  local url = sformat("http://overpass-api.de/api/interpreter?data=%s", query)
  return url
end
--[[
local lat_center, lon_center = unpack(center_lla, 1, 2)
local filename = sformat("/tmp/map%.7f_%.7f.osm", lat_center, lon_center)
local cmd = sformat('wget "%s" -O %s', url, filename)
-- os.execute(cmd)
]]
lib.get_osm = get_osm

return lib
