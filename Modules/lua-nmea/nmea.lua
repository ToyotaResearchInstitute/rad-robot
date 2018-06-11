local lib = {}

local nmea_keys, nmea_match_str = {
  'Timestamp',
  'Validity', -- A-ok, V-invalid
  'Latitude',
  'North_South',
  'Longitude',
  'East_West',
  'Speed', -- knots
  'Course',
  'Datestamp',
  'Variation',
  'Variation_EW',
}, {}
for i, _ in ipairs(nmea_keys) do
  nmea_match_str[i] = "([^,]*)"
end
nmea_match_str = '^$GPRMC,'..table.concat(nmea_match_str, ',')
-- Fix
-- A=autonomous, D=differential, E=Estimated, N=not valid, S=Simulator
nmea_match_str = nmea_match_str..',([ADENS]?)'
table.insert(nmea_keys, "Fix")

-- Add the checksum
nmea_match_str = nmea_match_str..'%*(%S%S)'
table.insert(nmea_keys, "Checksum")
--print(nmea_match_str)

local function parse(line, obj)
  if type(line)~='string' then return false, "Bad sentence" end
  if type(obj) ~= 'table' then obj = {} end
  obj.sentence = line
  local matches = {line:match(nmea_match_str)}
  for i,v in ipairs(matches) do
    local val
    local key = nmea_keys[i]
    if key=='Latitude' or key=='Longitude' then
      local d, m = v:match"(%d+)(%d%d%.%d+)"
      d, m = tonumber(d), tonumber(m)
      val = m and (d + m / 60) or false
    elseif key=='Checksum' then
      val = v -- Not number
    else
      val = tonumber(v) or v
    end
    obj[key] = val
  end
  return obj
end
lib.parse_nmea = parse

function lib.update(new_data)
  local str = ''
  while true do
    if type(new_data)=='string' then
      str = str..new_data
    end
    local istart = str:find('\n', 1, true)
    local obj
    if istart then
      local line = str:sub(1, istart-1)
      str = str:sub(istart + 1)
      obj = parse(line)
    end
    new_data = coroutine.yield(obj)
  end
end

return lib

