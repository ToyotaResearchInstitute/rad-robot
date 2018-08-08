local RadonTransform = {}

local ffi = require'ffi'
local fabs = require'math'.abs
local cos, sin = require'math'.cos, require'math'.sin
local min, max =  require'math'.min, require'math'.max

local function clear(self)
  -- Size of the zeroing
  local n_cells = self.NTH * self.MAXR
  local b_size32 = ffi.sizeof'int32_t' * n_cells
  -- Zero the counts
  ffi.fill(self.count_d, b_size32)
  ffi.fill(self.line_sum_d, b_size32)
  -- Fill up the min/max lines
  for ith=0,self.NTH-1 do
    for ir=0,self.MAXR-1 do
      self.line_min_d[ith][ir] = 2139062143
      self.line_max_d[ith][ir] = -2139062140
    end
  end
  return self
end

local function get_line_segment(self, ith, ir)
  if not ith or not ir then return false, "Bad inputs" end
  local s, c = self.sin_d[ith], self.cos_d[ith]
  local iline_min = self.line_min_d[ith][ir]
  local i_min = c * ir - s * iline_min
  local j_min = s * ir + c * iline_min
  --
  local iline_max = self.line_max_d[ith][ir]
  local i_max = c * ir - s * iline_max
  local j_max = s * ir + c * iline_max
  --
  local iline_mean = self.line_sum_d[ith][ir] / self.count_d[ith][ir]
  local i_mean = c * ir - s * iline_mean
  local j_mean = s * ir + c * iline_mean
  --
  return {i_min, j_min}, {i_max, j_max}, {i_mean, j_mean}
end

local function addPixelToRay(self, ith, i, j)
  local s, c = self.sin_d[ith], self.cos_d[ith]
  -- Counts and Line statistics
  local iline = c * j - s * i
  local ir = c * i + s * j
  -- Using 2*NTH
  -- if ir < 0 then
  --   ir = -ir
  --   -- Flip the angle by 180
  --   ith = (ith >= self.NTH) and (ith - self.NTH) or (ith + self.NTH)
  -- end
  -- Not using 2*NTH
  ir = fabs(ir)
  -- Scale properly
  ir = ir / self.RSCALE
  local count = self.count_d[ith][ir] + 1
  self.count_d[ith][ir] = count
  self.line_sum_d[ith][ir] = self.line_sum_d[ith][ir] + iline
  self.line_min_d[ith][ir] = min(self.line_min_d[ith][ir], iline)
  self.line_max_d[ith][ir] = max(self.line_max_d[ith][ir], iline)
end

-- Input is an edge image - strength of the edge
-- TODO: bbox argument for the area of the image to focus on
local function lines_from_edges(self, edge_ptr, width, height, threshold)
  threshold = threshold or 1
  local NTH = self.NTH
  -- Clear out any old transform
  self:clear()
  -- Run through the pointer
  local e = edge_ptr
  for j=0, (height - 1) do
    for i=0, (width - 1) do
      local use_point = e[0] >= threshold
      if use_point then
        for ith=0,(NTH-1) do self:addPixelToRay(ith, i, j) end
      end
      e = e + 1
    end
  end
  return self
end

--[[
-- TODO: Use custom thresholding functions
local function simple_threshold(label_nw, label_ne, label_sw)
  local has_vert = fabs(label_nw - label_ne) >= threshold
  local has_horiz = fabs(label_nw - label_sw) >= threshold
  return has_vert, has_horiz
end
--]]
local function lines_from_image(self, edge_ptr, width, height, threshold)
  threshold = threshold or 1
  -- Use pixel directions
  local label_nw, label_ne, label_sw
  -- local label_se
  -- Clear out any old transform
  self:clear()
  local NTH, NTH_HALF = self.NTH, self.NTH_HALF
  -- Start the pointers
  local e_ptr_l = edge_ptr
  local e_ptr_r = e_ptr_l + width
  -- Loop is -2 since we do not hit the boundary with ptr+1
  for j=0, (height - 2) do
    for i=0, (width - 2) do
      label_nw = e_ptr_l[0]
      e_ptr_l = e_ptr_l + 1
      label_ne = e_ptr_l[0]
      label_sw = e_ptr_r[0]
      e_ptr_r = e_ptr_r + 1
      -- label_se = e_ptr_r[0]
      -- Strong zero crossings in the image
      local has_vert = fabs(label_nw - label_ne) >= threshold
      local has_horiz = fabs(label_nw - label_sw) >= threshold
      local a = has_vert and 0 or NTH_HALF
      local b = has_horiz and NTH or NTH_HALF
      for ith=a,(b-1) do self:addPixelToRay(ith, i, j) end
    end
    -- Must have one more increment to get to the next line, since -2
    e_ptr_l = e_ptr_l + 1
    e_ptr_r = e_ptr_r + 1
  end
  return self
end

function RadonTransform.init(MAXR, RSCALE)
  -- Scale to reduce the search space
  RSCALE = RSCALE or 2
  -- Max radius: the diagonal of the image
  -- Assume 160x120 default
  MAXR = math.ceil((tonumber(MAXR) or 200) / RSCALE)

  -- NOTE: Should be an even number
  --local NTH = 180 -- (1 degree res)
  --local NTH = 90 -- Number of angles (2 degree res)
  local NTH = 36 -- 5 degree resolution
  --local NTH = 30 -- 6 degree resolution
  local ITH_TO_TH = 180 / NTH
  local NTH_HALF = NTH / 2

  -- Save our lookup table discretization
  local th_d = ffi.new('double[?]', NTH)
  local sin_d, cos_d = ffi.new('double[?]', NTH), ffi.new('double[?]', NTH)
  for i = 0, NTH-1 do
    -- We only need 0 to Pi because of the periodicity of sin/cos
    local th = (math.pi / NTH) * i
    -- TODO: Change based on the prior
    th_d[i] = th
    cos_d[i] = cos(th)
    sin_d[i] = sin(th)
  end

  local storage_type = 'int32_t'
  local storage_array_str = string.format("%s[%d][%d]",
                                          storage_type, NTH, MAXR)

  local props = {
    -- Per image information
    count_d = ffi.new(storage_array_str),
    line_sum_d = ffi.new(storage_array_str),
    line_min_d = ffi.new(storage_array_str),
    line_max_d = ffi.new(storage_array_str),
    -- Initialized constants
    NTH = NTH,
    NTH_HALF = NTH_HALF,
    ITH_TO_TH = ITH_TO_TH,
    MAXR = MAXR,
    RSCALE = RSCALE,
    cos_d = cos_d,
    sin_d = sin_d,
    th_d = th_d,
    -- Methods
    addPixelToRay = addPixelToRay,
    clear = clear,
    lines_from_edges = lines_from_edges,
    lines_from_image = lines_from_image,
    get_line_segment = get_line_segment,
  }
  clear(props)
  return props
end

return RadonTransform
