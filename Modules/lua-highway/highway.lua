local math = require'math'
local lib = {}

-- Add an event, for instance:
-- change in the number of lanes, exit, oncoming lane pass zone
-- All markers after this event have the same properties, until a new event changes that
local function add_event(self, distance, name, info)
  -- Information: an optional table
  if not info then
    info = false
  elseif type(info)~='table' then
    return false, "Bad info"
  end
  -- Lua is 1-indexed
  local i_marker = math.floor(distance / self.marker_interval) + 1
  -- Grab and update the object
  local marker_events = self.markers[i_marker]
  -- Add at the correct, ordered, position
  -- TODO: Check if there is a simple consolidation
  local added_event = false
  for i_event=1,#marker_events do
    local d = marker_events[i_event][1]
    if distance < d then
      table.insert(marker_events, i_event, {distance, name, info})
      break
    end
  end
  if not added_event then
    -- Add to the end
    table.insert(marker_events, {distance, name, info})
  end
  -- Add the event to the lookup, based on the name of the event
  -- Add at the correct, ordered, position
  local named_event = self.events[name]
  if not named_event then
    named_event = {{distance, info}}
    self.events[name] = named_event
  else
    for i_event=1,#named_event do
      local d = named_event[i_event][1]
      if distance < d then
        table.insert(named_event, i_event, {distance, info})
        break
      end
    end
  end
  -- Return self, in case of chaining
  return self
end
lib.add_event = add_event


local function events_by_name(self, name)
  -- Given the event type
  return self.events[name]
end
local function events_at_marker(self, i_marker)
  -- Given the mile marker index
  if i_marker < 1 or i_marker > self.n_markers then
    return false, "Marker is out-of-bounds"
  end
  -- Ensure integer
  i_marker = math.floor(i_marker)
  return self.markers[i_marker]
end
local function events_at_distance(self, d, radius)
  -- Given the distance in kilometers, find nearby event within a certain radius
end


-- Make a new highway
function lib.new(options)
  -- length in kilometers. default of 100 km
	local length = tonumber(options.length) or 100
  -- marker intervals, in kilometers
  local marker_interval = tonumber(options.marker_interval) or 1.0
  -- Ensure the minimum length
  if length < marker_interval then
    return false, "Bad length / interval"
  end
  -- Find the number of mile markers to use
  local n_markers = math.floor(length / marker_interval)
  if n_markers > 2048 then
    return false, "Too fine of a discretization"
  end
  -- Each marker links to a list of events
  -- Default of no events simply means a single lane
  -- markers[i_marker] = {distance, information}
  local markers = {}
  for _ = 1, n_markers do
    table.insert(markers, {})
  end
  -- Save a lookup table of the events, keyed by name
  -- events[name] = {distance, information}
  local events = {}

  -- Maintain a table of self
  local obj = {
    length = length,
    marker_interval = marker_interval,
    n_markers = n_markers,
    markers = markers,
    events = events,
    -- Methods
    add_event = add_event,
    events_by_name = events_by_name,
    events_at_marker = events_at_marker,
    events_at_distance = events_at_distance,
  }
  return obj
end

return lib