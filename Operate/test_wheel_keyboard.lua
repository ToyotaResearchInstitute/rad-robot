-----------------------------------------------------------------
-- Keyboard Wizard
-- Listens to keyboard input to control the arm joints
-- (c) Stephen McGill, 2013
---------------------------------

dofile'include.lua'
--local is_debug = true

-- Libraries
local Config  = require'Config'
local unix  = require'unix'
local getch = require'getch'
local mp    = require'msgpack'
local util  = require'util'
local vector  = require'vector'
local Body  = require'Body'
-- Keypresses for walking
local simple_ipc = require'simple_ipc'
--local motion_events = simple_ipc.new_publisher('fsm_motion',true)
local rpc_ch = simple_ipc.new_requester(Config.net.reliable_rpc)

-- Events for the FSMs
local char_to_event = {
  ['7'] = {'MotionFSM','sit'},
  ['8'] = {'MotionFSM','stand'},
  ['9'] = {'MotionFSM','walk'},
  ['p'] = {'MotionFSM','step'},
  --
--  ['t'] = {'BodyFSM','teleop'},
  --
  ['a'] = {'ArmFSM','init'},
  ['s'] = {'ArmFSM','reset'},
  ['r'] = {'ArmFSM','ready'},
  --
  ['x'] = {'ArmFSM','teleop'},
  --
  ['w'] = {'ArmFSM','wheelgrab'},
  --
  ['d'] = {'ArmFSM','doorgrab'},
--  
  ['t'] = {'ArmFSM','toolgrab'},

}

local char_to_vel = { 
}

local char_to_wheel = {
  ['['] = vector.new({-3*Body.DEG_TO_RAD}),
  [']'] = vector.new({3*Body.DEG_TO_RAD}),
}

local char_to_wheelmodel={
  ['i'] = vector.new({0.01, 0, 0,  0,0,0}),
  [','] = vector.new({-.01, 0, 0,  0,0,0}),
  ['j'] = vector.new({0, 0.01, 0,  0,0,0}),
  ['l'] = vector.new({0, -0.01, 0, 0,0,0}),
  ['u'] = vector.new({0, 0, 0.01,  0,0,0}),
  ['m'] = vector.new({0, 0, -0.01, 0,0,0}),

  ['y'] = vector.new({0, 0, 0,  0,math.pi/180,0}),
  ['n'] = vector.new({0, 0, 0, 0,-math.pi/180,0}),


  ['='] = vector.new({0, 0, 0, 0,0, -0.01}),
  ['-'] = vector.new({0, 0, 0, 0,0, 0.01}),

--  ['h'] = vector.new({0, 0.1, 0}),
--  [';'] = vector.new({0, -.1, 0}),
}

local char_to_shoulderangle={
  ['f'] = vector.new({-3*Body.DEG_TO_RAD}),
  ['g'] = vector.new({3*Body.DEG_TO_RAD}),
}


local function send_command(cmd)
  -- Default case is to send the command and receive a reply
  local ret   = rpc_ch:send(mp.pack(cmd))
  local reply = rpc_ch:receive()
  return mp.unpack(reply)
end

local function process_character(key_code,key_char,key_char_lower)
  local cmd

  -- Send motion fsm events
  local event = char_to_event[key_char_lower]
  if event then
    print( event[1], util.color(event[2],'yellow') )
    cmd = {}
    cmd.fsm = event[1]
    cmd.evt = event[2]
    return send_command(cmd)
  end

--[[
  -- Adjust the velocity
  -- Only used in direct teleop mode
  local vel_adjustment = char_to_vel[key_char_lower]
  if vel_adjustment then
    print( util.color('Inc vel by','yellow'), vel_adjustment )
    cmd = {}
    cmd.shm = 'mcm' 
    cmd.segment = 'walk'
    cmd.key = 'vel'
    cmd.delta = vel_adjustment
    return send_command(cmd)
  elseif key_char_lower=='k' then
    print( util.color('Zero Velocity','yellow'))
    cmd = {}
    cmd.shm = 'mcm'
    cmd.segment = 'walk'
    cmd.key = 'vel'
    cmd.val = {0, 0, 0}
    return send_command(cmd)
  end
--]]
  local wheel_model = char_to_wheelmodel[key_char_lower]
  if wheel_model then
    print( util.color('Move wheel wheel','yellow'), wheel_model )
    cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'wheel'
    cmd.key = 'model'
    cmd.delta = wheel_model;    
    return send_command(cmd)
  elseif key_char_lower=='k' then
    print( util.color('Center the wheel model','yellow') )
    cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'wheel'
    cmd.key = 'model'
    cmd.val = {0.40,0,0.10,0,0,0.10}
    return send_command(cmd)
  end

  -- Adjust the wheel angle
  local wheel_adj = char_to_wheel[key_char_lower]
  if wheel_adj then
    print( util.color('Turn wheel','yellow'), wheel_adj )
    cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'wheel'
    cmd.key = 'turnangle'
    cmd.delta = wheel_adj
    return send_command(cmd)
  
  elseif key_char_lower=='\\' then
    print( util.color('Center the wheel','yellow') )
    cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'wheel'
    cmd.key = 'turnangle'
    cmd.val = {0}
    return send_command(cmd)
  end

  local shoulder_adj = char_to_shoulderangle[key_char_lower]
  if shoulder_adj then
    print( util.color('Shoulder yaw adj','yellow'), shoulder_adj )
    cmd = {}
    cmd.shm = 'hcm'
    cmd.segment = 'joints'
    cmd.key = 'shoulderangle'
    cmd.delta = shoulder_adj
--    print(shoulder_adj)
    return send_command(cmd)
  end

  -- TODO: smarter range setting
  -- For now, care about things from 10cm to 1m in front
  local near, far = 0.10, 1
  if key_char_lower=='v' then
    print( util.color('Request Head Mesh','yellow') )
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'head_lidar'
    cmd.key = 'depths'
    cmd.val = {near,far}
    send_command(cmd)
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'head_lidar'
    cmd.key = 'net'
    --cmd.val = {1,1,0} --jpeg
    cmd.val = {1,2,0} --zlib
    return send_command(cmd)
  elseif key_char_lower=='c' then
    print( util.color('Request Chest Mesh','yellow') )
    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'chest_lidar'
    cmd.key = 'depths'
    cmd.val = {near,far}
    send_command(cmd)

    cmd = {}
    cmd.shm = 'vcm'
    cmd.segment = 'chest_lidar'
    cmd.key = 'net'
    --cmd.val = {1,1,0} --jpeg
    cmd.val = {1,2,0} --zlib
    return send_command(cmd)
  end

end

------------
-- Start processing
os.execute("clear")
io.flush()
local t0 = unix.time()

-------------------------------
-- Initialize HSM here
cmd = {}
cmd.shm = 'hcm'
cmd.segment = 'wheel'
cmd.key = 'model'
cmd.val = {0.40,0,0.10, 0,0,0.10}
send_command(cmd)

cmd.segment = 'wheel'
cmd.key = 'turnangle'
cmd.val = {0}
send_command(cmd)

cmd.segment = 'joints'
cmd.key = 'shoulderangle'
cmd.val = {45*Body.DEG_TO_RAD}
send_command(cmd)


local char_to_shoulderangle={
  ['c'] = vector.new({-3*Body.DEG_TO_RAD}),
  ['v'] = vector.new({3*Body.DEG_TO_RAD}),
}


----------------------------------

while true do
  
  -- Grab the keyboard character
  local key_code = getch.block()
  local key_char = string.char(key_code)
  local key_char_lower = string.lower(key_char)
  
  -- Process the character
  local msg = process_character(key_code,key_char,key_char_lower)
  
  -- Measure the timing
  local t = unix.time()
  local t_diff = t-t0
  t0 = t
  local fps = 1/t_diff
  
  -- Print is_debugging message
  if is_debug then
    print( string.format('\nKeyboard | Code: %d, Char: %s, Lower: %s',
    key_code,key_char,key_char_lower) )
    print('Response time:',t_diff)
  end
    
end
