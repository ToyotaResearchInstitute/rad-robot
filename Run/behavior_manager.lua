---------------------------------
-- Behavior manager for Team THOR
-- (c) Stephen McGill, 2013
---------------------------------

-- Set the path for the libraries
dofile('include.lua')
-- Set the Debugging mode
local debug = true;

-- Libraries
local simple_ipc = require 'simple_ipc'
require 'unix'
require 'cjpeg'

-- For the motion manager interaction
require('rpc')
local motion_manager_endpoint = 'tcp://localhost:12001'
local motion_manager = rpc.client.new(motion_manager_endpoint)
print('Attempting to connect to motion_manager at '..motion_manager_endpoint)
motion_manager:connect(nil)
motion_manager:set_timeout(0.1)

-- Setup IPC Channels
local omap_channel   = simple_ipc.new_subscriber('omap');
local oct_channel    = simple_ipc.new_subscriber('oct');
local hmi_channel    = simple_ipc.new_subscriber('hmi');

-- Replan based on occupancy map
omap_channel.callback = function()
  local omap_data, has_more = omap_channel:receive()
  if locomotion_state=='walk' then
    print('walking with the omap guidance!')
    motion_manager:call('walk:set_velocity', unpack(desired_velocity))
  end
  -- TODO: Get the timestamp
  print('OMAP | ', 'Replanned waypoints!')
end

-- Replan based on Oct Tree
oct_channel.callback = function()
  local oct_data, has_more = oct_channel:receive()
  -- TODO: Get the timestamp
  print('Oct | ', 'Replanned manipulation targets!')
end

-- Replan based on HMI
hmi_channel.callback = function()
  local hmi_data, has_more = hmi_channel:receive()
  -- TODO: Get the timestamp
  print('HMI | ', 'Replanned waypoints!')
end

-- Poll multiple sockets
local wait_channels = {omap_channel, oct_channel, hmi_channel}
local channel_poll = simple_ipc.wait_on_channels( wait_channels );
local channel_timeout = 100; -- 100ms timeout

-- Start the timing
local t = unix.time();
local t_last = t;
local t_debug = 1; -- Print debug output every second

-- No debugging messages
-- Only the callback will be made
if not debug then
  channel_poll:start()
end

local cnt = 0;
while true do
  local npoll = channel_poll:poll(channel_timeout)

  -- Upon completion of the poll, track the robot state
  local success, state = motion_manager:call('Locomotion:get_state')
  -- Ensure that the robot is alway
  if (state ~= 'stand') then
    motion_manager:call('Locomotion:add_event', 'stand')
  else
    motion_manager:call('Locomotion:add_event', 'walk')
  end
  
  
  local t = unix.time()
  cnt = cnt+1;
  if t-t_last>t_debug then
    local msg = string.format("%.2f FPS", cnt/t_debug);
    print("Behavior Manager | "..msg)
    t_last = t;
    cnt = 0;
  end
end
