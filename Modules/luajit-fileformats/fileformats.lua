local lib = {}
local has_ffi, ffi = pcall(require, 'ffi')
local has_unix, unix = pcall(require, 'unix')
local min, max = require'math'.min, require'math'.max
local floor = require'math'.floor
local sformat = require'string'.format
local tconcat = require'table'.concat
local tinsert = require'table'.insert
local unpack = unpack or require'table'.unpack

local function get_cdata_dims(arr)
  local dims = {}
  for d in tostring(ffi.typeof(arr)):gmatch"%[(%d+)%]" do
    tinsert(dims, tonumber(d))
  end
  return dims
end

function lib.load_netpbm(fname)
  if type(fname)~='string' then
    return false, "Bad filename"
  end
  local f_pgm
  if fname:match"%.gz$" then
    f_pgm = io.popen("gzip -dc "..fname)
  else
    f_pgm = io.open(fname)
  end
  if not f_pgm then
    return false, "Could not open"
  end

  local state = "magic"
  local comments = {}
  local magic, width, height, maxval
  repeat
    local line = f_pgm:read"*line"
    if line:match("^#") then
      tinsert(comments, line)
    elseif state=='magic' then
      magic = line
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
      maxval = tonumber(line)
      state = 'data'
    end
  until state == "data"
  if magic ~="P5" then return false, "Bad magic" end
  if maxval~=255 then return false, "Bad maxval" end
  local n_cells = width * height
  local n_channels = 1
  local map
  if magic=='P5' then
    if has_ffi and has_unix then
      map = ffi.new("uint8_t[?]", n_cells)
      unix.fread(f_pgm, map, n_cells)
    end
  elseif magic=='P2' then
    map = {}
    for l in f_pgm:lines() do
      -- gmatch each number
      local line = {}
      for num in l:gmatch"%d+" do
        tinsert(line, tonumber(num))
      end
      tinsert(map, line)
    end
  elseif magic=='P6' then
    -- RGB
    n_channels = 3
    if has_ffi and has_unix then
      map = ffi.new("uint8_t[?]", 3 * n_cells)
      unix.fread(f_pgm, map, 3 * n_cells)
    end
  elseif magic=='P3' then
    -- RGB
    n_channels = 3
    map = {}
    for l in f_pgm:lines() do
      -- gmatch each number
      local line = {}
      for r,g,b in l:gmatch"(%d+) (%d+) (%d+)" do
        -- TODO: Packed or planar
        tinsert(line, {tonumber(r), tonumber(g), tonumber(b)})
      end
      tinsert(map, line)
    end
  end
  f_pgm:close()
  return map, {width, height, n_channels}, comments
end

function lib.save_netpbm(fname, map, width, height, comments, maxval)
  local f_pgm
  if io.type(fname)=='file' then
    f_pgm = fname
  elseif fname:match"%.gz$" then
    f_pgm = io.popen("gzip > "..fname, "w")
  else
    f_pgm = io.open(fname, "w")
  end
  if not f_pgm then return false end
  if type(comments)~='table' then
    comments = {}
  end
  maxval = max(0, min(tonumber(maxval) or 255, 255))
  if type(map) == 'table' then
    local header = {
      "P2",
    }
    for k, v in pairs(comments) do
      local comment = string.format("# %s: %s", tostring(k), tostring(v))
      tinsert(header, comment)
    end
    tinsert(header, sformat("%d %d", width, height))
    tinsert(header, sformat("%d", maxval))
    tinsert(header, '')
    f_pgm:write(tconcat(header, "\n"))
    local line = {}
    for row in map do
      for i, el in ipairs(row) do
        line[i] = max(0, min(maxval, floor(el)))
      end
      f_pgm:write(tconcat(line, " "), "\n")
    end
  else
    local header = {
      "P5",
    }
    for k, v in pairs(comments) do
      tinsert(header, sformat('# %s: %s', tostring(k), tostring(v)))
    end
    tinsert(header, sformat("%d %d", width, height))
    tinsert(header, sformat("%d", maxval))
    tinsert(header, "")
    local hdr = tconcat(header, "\n")
    -- f_pgm:write(hdr, "\n")
    unix.fwrite(f_pgm, hdr, #hdr)
    unix.fwrite(f_pgm, map, width * height)
  end
  if io.type(fname)=='file' then
    return true
  else
    return f_pgm:close()
  end
end

local function ply_header(n)
  -- https://courses.cs.washington.edu/courses/cse558/01sp/software/scanalyze/points.html
  n = n or 0 -- can go back and overwrite
  local tbl = {
    "ply",
    "format ascii 1.0",
    sformat("element vertex %10d", n),
    "property float x",
    "property float y",
    "property float z",
    -- Can add other properties
    "end_header"
  }
  local hdr = tconcat(tbl, '\n')
  -- Yield the offset for overwriting the vertex count
  local _, ind = hdr:find"element vertex "
  return hdr, ind
end

-- Point cloud
function lib.load_ply(fname, points, is_planar)
  -- PLY is a header followed by an xyz file tuple
  local f_ply
  if fname:match"%.gz$" then
    f_ply = io.popen("gzip -dc "..fname)
  else
    f_ply = io.open(fname)
  end
  if not f_ply then return false, "Bad filename" end
  local props = {}
  local n_pts
  for l in f_ply:lines() do
    if l:match"^format" then
      local fmt = l:match"^format%s+(%S+)%s+1.0$"
      print("Format", fmt)
    elseif l:match"^element" then
      local _, n = l:match"^element%s+(%S+)%s+(%S+)"
      n_pts = tonumber(n)
    elseif l:match"^property" then
      local _, name = l:match"^property%s+(%S+)%s+(%S+)"
      tinsert(props, name)
    elseif l=="end_header" then break end
  end
  -- Now read all the points
  print(sformat("%d points of dimension %d", n_pts, #props))
  local use_cb = type(points)=='function'
  if not use_cb then
    local tp = type(points)
    if tp~='table' and tp~='cdata' then points = {} end
    if is_planar then
      for i=1,#props do points[i] = points[i] or {} end
    end
  end
  for l in f_ply:lines() do
    local row = {}
    for v in l:gmatch"%S+" do row[#row+1] = tonumber(v) end
    -- Callback on the row...
    if use_cb then
      points(row)
    elseif is_planar then
      for i=1,#props do
        local p = points[i]
        p[#p+1] = row[i]
      end
    else
      tinsert(points, row)
    end
  end
  f_ply:close()
  return points
end
function lib.save_ply(fname, points, is_planar)
  -- PLY is a header followed by an xyz file tuple
  local f_ply, is_gz
  if fname:match"%.gz$" then
    is_gz = true
    f_ply = io.popen("gzip > "..fname, "w")
  else
    is_gz = false
    f_ply = io.open(fname, "w")
  end
  if not f_ply then return false, "Bad filename" end
  -- For each type of input
  local tp = type(points)
  if tp=='function' and is_gz then
    -- Streaming list of points
    local f_xyz = io.tmpfile()
    local n = 0
    for p in points do
      local line = tconcat(p, " ")
      f_xyz:write(line, '\n')
      n = n + 1
    end
    f_ply:write(ply_header(n), "\n")
    -- Copy the xyz file
    f_xyz:seek("set")
    for l in f_xyz:lines() do f_ply:write(l, '\n') end
    f_xyz:close()
  elseif tp=='function' then
    -- New method: supports up to, but excluding, 10 billion points
    local hdr, offset = ply_header()
    f_ply:write(hdr, "\n")
    local n = 0
    for p in points do
      local line = tconcat(p, " ")
      f_ply:write(line, '\n')
      n = n + 1
      if n>=1e10 then
        io.stderr:write(sformat("More than %d >= 1e10\n", n))
        break
      end
    end
    f_ply:seek("set", offset)
    f_ply:write(sformat("%10d", n))
  elseif tp=='table' and is_planar then
    -- Planar list of points given
    local nd = #points -- Number of dimensions
    local np = #points[1]
    f_ply:write(ply_header(np), "\n")
    local line = {}
    for i=1,np do
      -- for each plane dimension
      for j=1,nd do line[j] = points[j][i] end
      f_ply:write(tconcat(line, " "), '\n')
    end
  elseif tp=='table' then
    -- Packed list of points given
    local n = #points
    f_ply:write(ply_header(n), "\n")
    for _, p in ipairs(points) do
      local line = tconcat(p, " ")
      f_ply:write(line, '\n')
    end
  elseif tp=='cdata' and is_planar then
    -- Planar list of points given
    local dims = get_cdata_dims(points)
    local nd, np = unpack(dims, 2)
    f_ply:write(ply_header(np), "\n")
    local line = {}
    for i=0,np-1 do
      for j=0,nd-1 do line[i+1] = points[j][i] end
      f_ply:write(tconcat(line, ' '), '\n')
    end
  elseif tp=='cdata' then
    -- Packed list of points given
    local dims = get_cdata_dims(points)
    local np, nd = unpack(dims, 2)
    f_ply:write(ply_header(np), "\n")
    local line = {}
    for i=0,np-1 do
      for j=0,nd-1 do line[i+1] = points[i][j] end
      f_ply:write(tconcat(line, ' '), '\n')
    end
  end
  f_ply:close()
  return true
end

return lib
