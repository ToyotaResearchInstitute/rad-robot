#!/usr/bin/env luajit
local grid = require'grid'

local my_grid = assert(grid.new{
  scale = 0.05,
  xmin = -10, xmax = 10,
  ymin = -10, ymax = 10
  })

local function set_mid(map, idx)
  map[idx] = 127
end

my_grid:bresenham({my_grid.xmin, my_grid.ymin}, {my_grid.xmax, my_grid.ymax})
--
my_grid:bresenham({my_grid.xmin, my_grid.ymin}, {0, my_grid.ymax})
my_grid:bresenham({my_grid.xmin, my_grid.ymin}, {my_grid.xmax, 0})
--
my_grid:bresenham({my_grid.xmin, my_grid.ymin}, {-5, my_grid.ymax})
my_grid:bresenham({my_grid.xmin, my_grid.ymin}, {my_grid.xmax, -5})
--
my_grid:bresenham({0, 0}, {0, my_grid.ymax})
my_grid:bresenham({my_grid.xmin, 0}, {my_grid.xmax, 0})
-- Check that the order doesn't matter
my_grid:bresenham({my_grid.xmax, my_grid.ymin}, {my_grid.xmin, my_grid.ymax})
my_grid:bresenham({my_grid.xmax, my_grid.ymin}, {0, my_grid.ymax})
my_grid:bresenham({my_grid.xmax, my_grid.ymin}, {-5, my_grid.ymax})
--
my_grid:bresenham({my_grid.xmax, my_grid.ymax}, {0, my_grid.ymin})
my_grid:bresenham({my_grid.xmax, my_grid.ymax}, {-5, my_grid.ymin})
--
my_grid:bresenham({my_grid.xmax, my_grid.ymax}, {my_grid.xmin, 0})
my_grid:bresenham({my_grid.xmax, my_grid.ymax}, {my_grid.xmin, -5})
--
my_grid:bresenham({my_grid.xmax, my_grid.ymin}, {my_grid.xmin, 0})
my_grid:bresenham({my_grid.xmax, my_grid.ymin}, {my_grid.xmin, -5})

-- Write an arc
my_grid:arc({0, 0}, 3, 0, math.pi)

my_grid:circle({-5, -5}, 4, set_mid)

assert(my_grid:save"test.pgm")

local my_grid1 = assert(grid.new{fname="test.pgm"})

for k, v in pairs(my_grid1) do
  if type(v)=='number' then
    print(k, v)
  end
end

local potential_grid = assert(grid.new{
  scale = 1,
  xmin = 0, xmax = 60,
  ymin = 0, ymax = 80,
  datatype='double'
  })

local r_obs = 10
potential_grid:potential_field({75, 55}, {{40, 30, r_obs}, {20, 15, r_obs}, {60, 45, r_obs}})
potential_grid:save("test_potential.pgm")

grid.new({
  scale = 1,
  xmin = 0, xmax = 60,
  ymin = 0, ymax = 80,
  }):voronoi({
    {75, 55},{40, 30, r_obs}, {20, 15, r_obs}, {60, 45, r_obs}
  }):save("test_voronoi.pgm", {use_max = true})
