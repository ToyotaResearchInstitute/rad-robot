#!/usr/bin/env luajit
local unpack = unpack or require'table'.unpack
local highway = require'highway'

local interstate95 = highway.new{
  length = 517, -- length in kilometers
  marker_interval = 1.0 -- marker intervals, in kilometers
}

local interstate76 = highway.new{
  length = 37.81958, -- length in kilometers
  marker_interval = 1.0 -- marker intervals, in kilometers
}

local my_highways = {
  ['i95'] = interstate95,
  ['i76'] = interstate76,
}

-- Add an exit _to_ I-76 at 100 kilometers
assert(interstate95:add_event(101.5, "exit", {
  id = 1, -- Exit 1
  highway = {'i76', 25.25} -- the connecting highway, id and distance
}))
-- Add an entrance _from_ I-76
assert(interstate95:add_event(101.6, "entrance", {
  id = 1, -- Entrance 1
  highway = {'i76', 25.25} -- the connecting highway, id and distance
}))

-- Add a lane before the exit
assert(interstate95:add_event(101.1, "lane", {
  n = 3
}))

-- One exiting on the left side (US), right side (UK/JPN)
assert(interstate95:add_event(120.0, "exit", {
  on_far_side = true
}))

-- Iterate through all of the mile markers
for i_marker, events in ipairs(interstate95.markers) do
  if #events > 0 then
    print("Mile Marker (km)", i_marker)
  end
  for i_event, event in ipairs(events) do
    print("== Event", i_event)
    local dist, name, info = unpack(event)
    print(dist, name)
    if info then
      for kk, vv in pairs(info) do
        print("--", kk, vv)
      end
    end
  end
end

-- Print all of the exits
local evt_name = "exit"
print("Finding all", evt_name)
for _, evt in ipairs(interstate95:events_by_name(evt_name)) do
  print(unpack(evt))
end