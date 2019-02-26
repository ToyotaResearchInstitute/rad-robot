#!/usr/bin/env luajit

local lat0 = 39.9525
local lon0 = -75.1652

print("LLA0", lat0, lon0)

local proj = require'proj'
local pr = proj.new{lat0, lon0}
print(pr:lla(-1, 1))
print(pr:lla(-10, 10))
print(pr:lla(-100, 100))
print(pr:enu(39.9525, -75.1652))
print(pr:enu(39.953, -75.166))
print(pr:enu(39.952, -75.164))

print("Dist", pr:geodesic({39.9525, -75.1652}, {39.952, -75.164}))

print("Dist", pr:geodesic({pr:lla(-1, 1)}, {pr:lla(1, -1)}))
print("Dist", pr:geodesic({pr:lla(0, 0)}, {pr:lla(100, 0)}))
print("Dist", pr:geodesic({pr:lla(0, 0)}, {pr:lla(1e3, 0)}))
print("Dist", pr:geodesic({pr:lla(0, 0)}, {pr:lla(0, 1e3)}))
print("Dist", pr:geodesic({pr:lla(0, 0)}, {pr:lla(1e4, 0)}))
print("Dist", pr:geodesic({pr:lla(0, 0)}, {pr:lla(0, 1e4)}))

print(proj.get_osm({lat0, lon0}, 1e3))

if true then return end

-- Project our raods files
lat0 = 42.3448
lon0 = -71.0520
P = ffi.gc(pjt.pjt_init(lat0, lon0), pjt.pjt_exit)

local mp = require'MessagePack'
local file = table.concat{
  "/tmp/map39.9525000_-75.1652000",
  ".bounds.mp"
}
local bounds = mp.unpack(io.open(file):read"*all")
lla[0], lla[1] = bounds.minlat, bounds.minlon
pjt.lla_to_enu(P, lla, enu)
local xmin, ymin = enu[0], enu[1]
print("xmin, ymin", xmin, ymin)
--
lla[0], lla[1] = bounds.maxlat, bounds.maxlon
pjt.lla_to_enu(P, lla, enu)
local xmax, ymax = enu[0], enu[1]
print("xmax, ymax", xmax, ymax)

local grid = require'grid'
local my_grid = assert(grid.new{
  scale = 1,
  xmin = xmin, xmax = xmax,
  ymin = ymin, ymax = ymax
})
print("n_cells", my_grid.n_cells)

local file = table.concat{
  "/tmp/map39.9525000_-75.1652000",
  ".segments.mp"
}

local segments = mp.unpack(io.open(file):read"*all")

-- local nSegments = bounds.nSegments

-- print(segments)
local cnt = 0
for k, v in pairs(segments) do
  cnt = cnt + 1
  -- print("k", k)
  -- print("Drawing segment", cnt, nSegments)
  local pts = v.points
  for i=2,#pts do
    -- Convert
    lla[0], lla[1] = unpack(pts[i-1])
    pjt.lla_to_enu(P, lla, enu)
    local ptA = {enu[0], enu[1]}
    lla[0], lla[1] = unpack(pts[i])
    pjt.lla_to_enu(P, lla, enu)
    local ptB = {enu[0], enu[1]}
    my_grid:bresenham(ptA, ptB)
  end
end

print("Saving")
my_grid:save"test_road.pgm"
