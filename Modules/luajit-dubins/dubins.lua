local tinsert = require'table'.insert
local ffi = require'ffi'

ffi.cdef[[
typedef enum
{
    LSL = 0,
    LSR = 1,
    RSL = 2,
    RSR = 3,
    RLR = 4,
    LRL = 5
} DubinsPathType;
typedef struct
{
    /* the initial configuration */
    double qi[3];
    /* the lengths of the three segments */
    double param[3];
    /* model forward velocity / model angular velocity */
    double rho;
    /* the path type described */
    DubinsPathType type;
} DubinsPath;
int dubins_shortest_path(DubinsPath* path, double q0[3], double q1[3], double rho);
double dubins_path_length(DubinsPath* path);
int dubins_path_sample(DubinsPath* path, double t, double q[3]);
]]
local dubins = ffi.load'dubins'

local lib = {}

local function shortest_path(from, to, TURNING_RADIUS)
  local path = ffi.new"DubinsPath"
  local q0 = ffi.new("double[3]", from)
  local q1 = ffi.new("double[3]", to)
  local ret = dubins.dubins_shortest_path(
      path, q0, q1, TURNING_RADIUS)
  if ret~=0 then
    return false, "Bad Dubins path"
  end
  return path
end
lib.shortest_path = shortest_path

local function path_length(path)
  return dubins.dubins_path_length(path)
end
lib.path_length = path_length

local function path_sample(path, t)
  local q = ffi.new("double[3]")
  dubins.dubins_path_sample(path, t, q)
  local xya = {q[0], q[1], q[2]}
  return xya
end
lib.path_sample = path_sample

-- TODO: Function to recover the whole path
local function path_list(path, a, b, step_size, path_tbl)
  if type(path_tbl) ~= 'table' then
    path_tbl = {}
  end
  for t=a,b,step_size do
    local xya = dubins.dubins_path_sample(path, t)
    tinsert(path_tbl, xya)
  end
  return path_tbl
end
lib.path_list = path_list

return lib