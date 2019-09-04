local math = require'math'
local unpack = unpack or require'table'.unpack
local lib = {}


local function find_in_highway(self, p_vehicle, options)
  options = options or {}
  local closeness = tonumber(options.closeness) or 1
  local orientation_threshold = tonumber(options.orientation_threshold) or math.rad(45)
  -- Grab the pose angle
  local p_x, p_y, p_a = unpack(p_vehicle)

  -- Find the marker - based on the x position, only
  local i_marker = self:get_marker(p_x)
  -- TODO: Find the lane
  -- local marker = self.markers[i_marker]
  -- local lanes_running = {unpack(marker.lanes)}
  -- local distances_running = {}
  -- local dist_marker = (i_marker - 1) * marker_interval
  -- for i=1,#lanes_running do
  --   table.insert(distances_running, dist_marker)
  -- end
  -- local marker_events = self.marker_events[i_marker]
  -- for _, evt in ipairs(marker_events) do
  --   local evt_dist, evt_name, evt_info = unpack(evt)
  --   -- No need to consider later events
  --   if evt_dist > p_x then break end
  --   if evt_name == "add_lane" then
  --     if evt_info.on_far_side then
  --       table.insert(lanes_running, 1, lanes_running[1] + 1)
  --       table.insert(distances_running, 1, evt_dist)
  --     else
  --       table.insert(lanes_running, lanes_running[#lanes_running] - 1)
  --       table.insert(distances_running, evt_dist)
  --     end
  --   elseif evt_name == "del_lane" then
  --     if evt_info.on_far_side then
  --       local y_begin = table.remove(lanes_running, 1)
  --       local x_begin = table.remove(distances_running, 1)
  --     else
  --       local y_begin = table.remove(lanes_running)
  --       local x_begin = table.remove(distances_running)
  --     end
  --   end
  -- end
  -- round to the nearest lane
  local i_lane = self:get_lane(p_y)
  local dy = p_y - i_lane * self.lane_width

  return {
    i_marker=i_marker,
    i_lane = i_lane,
    dist = math.abs(dy),
    -- Highway angle is offset
    da=p_a
  }

  
end
lib.find = find_in_highway

local function tostring_highway(hw)
  return string.format("Highway with %d events", hw.n_events)
end

local function export(hw)
  local evts = {}
  for _, v in pairs(hw.events) do
    for _, evt in ipairs(v) do
      table.insert(evts, evt)
    end
  end
  print("n_events exported", #evts, "of", hw.n_events)
  return {
    length = hw.length,
    marker_interval = hw.marker_interval,
    events = evts
  }
end

local function get_marker(self, distance)
  local index0 = math.floor(distance / self.marker_interval)
  -- Lua is 1-indexed
  return index0 + 1
end

local function get_lane(self, p_y)
  -- Lanes begin on the right, so floor to the small number,
  -- given right hand rule
  return math.floor(p_y / self.lane_width)
end



-- Add an event, for instance:
-- change in the number of lanes, exit, oncoming lane pass zone
-- All markers after this event have the same properties, until a new event changes that
-- Info: an optional table of information
local function add_event(self, evt)

  local distance = assert(tonumber(evt.meters or evt[1]), "Bad meter marker!")
  local evt_name = assert(evt.name or evt[2], "No name of event!")
  assert(type(evt_name)=='string', "Bad name of event!")
  local info = evt.info or evt[3] or {}
  assert(type(info)=='table', "Bad information for event!")
  -- Form our canonical event to track
  evt = {distance, evt_name, info}
  local i_marker = self:get_marker(distance)
  -- Grab and update the object
  local marker = self.markers[i_marker]
  local marker_events = marker.events
  -- Add at the correct, ordered, position
  -- TODO: Check if there is a simple consolidation
  local added_event = false
  for i_event=1,#marker_events do
    local d = marker_events[i_event][1]
    if distance < d then
      table.insert(marker_events, i_event, evt)
      break
    end
  end
  if not added_event then
    -- Add to the end
    table.insert(marker_events, evt)
  end
  -- Add the event to the lookup, based on the name of the event
  -- Add at the correct, ordered, position
  local named_event = self.events[evt_name]
  if not named_event then
    named_event = {evt}
    self.events[evt_name] = named_event
  else
    local did_insert = false
    for i_event=1,#named_event do
      local d = named_event[i_event][1]
      if distance < d then
        did_insert = true
        table.insert(named_event, i_event, evt)
        break
      end
    end
    if not did_insert then
      table.insert(named_event, evt)
    end
  end
  -- Track the number of events in the highway
  self.n_events = self.n_events + 1

  -- Now, update the marker properties
  -- NOTE: This is incorrect, but that is OK, for now
  for i_next = i_marker + 1, self.n_markers do
    local marker_next = self.markers[i_next]
    -- If a speed_limit, then set the precondition for the next lane
    if evt_name=='speed_limit' then
      if marker_next.speed_limit then break end
      local kph = tonumber(info.kph)
      marker_next.speed_limit = kph
    elseif evt_name=='add_lane' then
      -- Propogate forward
      if info.on_far_side then
        -- Adding on the far side is in the positive y direction
        local y_extreme_max = marker_next.lanes[1]
        -- Far side is the front of the list. Then we are more efficient in Lua land
        table.insert(marker_next.lanes, 1, y_extreme_max + 1)
      else
        -- Adding on the near side is in the negative y direction
        local y_extreme_min = marker_next.lanes[#marker_next.lanes]
        -- Add to the end of the queue.
        -- Efficient, since most lane changes are on near side
        table.insert(marker_next.lanes, y_extreme_min - 1)
      end
      -- End addition
    elseif evt_name=='del_lane' then
      if info.on_far_side then
        -- Far side is the front of the list. Then we are more efficient in Lua land
        table.remove(marker_next.lanes, 1)
      else
        -- Add to the end of the queue.
        -- Efficient, since most lane changes are on near side
        table.remove(marker_next.lanes)
      end
      -- End removal
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
  -- length in meters
	local length = assert(tonumber(options.length), "No highway length given")
  -- marker intervals, in meters
  local marker_interval = assert(tonumber(options.marker_interval), "No highway marker_interval given")
  -- Ensure the minimum length
  if length < marker_interval then
    return false, "Bad length / interval"
  end
  -- Find the number of mile markers to use
  local n_markers = math.floor(length / marker_interval)
  if n_markers > 2048 then
    return false, "Too fine of a discretization"
  end
  do
    local n_markers0 = tonumber(options.n_markers)
    if n_markers0 and n_markers0~=n_markers then
      return false, "Bad n_markers checksum"
    end
  end
  -- Each marker_events links to a list of events
  -- Default of no events simply means a single lane
  -- marker_events[i_marker] = {distance, information}
  -- Save a lookup table of the events, keyed by name
  -- events[name] = {distance, information}
  -- Properties as a pre condition for entering the marker
  -- Given this information, a user integrates events until the point of the vehicle
  -- Initial properties
  local lane_width = tonumber(options.lane_width) or 2
  local initial = options.initial or {}
  -- Meters per second
  local speed_limit0 = tonumber(initial.speed_limit) or 25
  local nlanes0 = tonumber(initial.lanes) or 2
  local lanes0 = {}
  for ilane=1,nlanes0 do
    table.insert(lanes0, 1, ilane - 1)
  end
  local markers = {}
  for _ = 1, n_markers do
    local props = {
      speed_limit = speed_limit0,
      lanes = {unpack(lanes0)},
      events = {}
    }
    table.insert(markers, props)
  end

  -- Maintain a table of self
  local obj = {
    length = length,
    marker_interval = marker_interval,
    n_markers = n_markers,
    n_events = 0,
    markers = markers,
    events = {},
    lane_width = lane_width,
    -- Methods
    add_event = add_event,
    events_by_name = events_by_name,
    events_at_marker = events_at_marker,
    events_at_distance = events_at_distance,
    find = find_in_highway,
    export = export,
    get_marker = get_marker,
    get_lane = get_lane,
    find = find_in_highway
  }

  -- Add events, if given
  if type(options.events) == 'table' then
    for _, evt in ipairs(options.events) do
      assert(obj:add_event(evt))
    end
    assert(obj.n_events==#options.events, "Bad digest")
  end
  do
    local n_events0 = tonumber(options.n_events)
    if n_events0 and n_events0~=obj.n_events then
      return false, string.format("Bad n_events checksum: %s != %s", n_events0, obj.n_events)
    end
  end

  return setmetatable(obj, {
    __tostring = tostring_highway
  })
end

local function wrap(obj)
  -- Add methods to the table
  obj.add_event = add_event
  obj.events_by_name = events_by_name
  obj.events_at_marker = events_at_marker
  obj.events_at_distance = events_at_distance
  obj.export = export
  obj.get_marker = get_marker
  obj.get_lane = get_lane
  obj.find = find_in_highway
  return setmetatable(obj, {
    __tostring = tostring_highway
  })
end
lib.wrap = wrap

return lib