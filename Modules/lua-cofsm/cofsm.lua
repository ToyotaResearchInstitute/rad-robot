local fsm = {}

--[[
-- Usage
-- New FSM from a table of transitions:
local botFsm = fsm.new{
  {'botStop', 'go', 'botGo'},
  {'botGo', 'stop', 'botStop'},
}

local function botGo()
  print('Entry', 'ok')
  local distance = 0
  local evt
  while coroutine.yield(evt) do
    print('Update', 'go', distance)
    distance = distance + 1
    -- error('oops')
    evt = distance > 3 and 'stop'
  end
  print('Exit', 'woo')
  return 'bye'
end
local function botStop()
  print('Entry', 'eek')
  local distance = 3
  local evt
  while coroutine.yield(evt) do
    print('Update', 'whoa', distance)
    distance = distance - 1
    evt = distance <= 0 and 'go'
  end
  print('Exit', 'phew')
  return 'hi'
end

-- Associate a coroutine function with a state
botFsm:add_state('botGo', botGo)

-- Add states after construction
botFsm:add_state('botThink')

-- Set transitions after construction
botFsm:add_transition('botStop', 'think', 'botThink')
botFsm:add_transition('botThink', 'done', 'botStop')

print('Initial State:', botFsm.current_state)
local evt
for i = 1, 20 do
  print(i, 'Current State:', botFsm.current_state)
  status, evt = botFsm:update(evt)
  print("Status/event", status, evt)
end
print('Final State:', botFsm.current_state)

--]]

local unpack = require'table'.unpack
local cocreate, coresume, costatus
local has_coroutine, coroutine = pcall(require, 'coroutine')
if has_coroutine then
  cocreate = coroutine.create
  coresume = coroutine.resume
  costatus = coroutine.status
end

-- This brings the FSM into an uninitialized state
-- Useful as a final call, or a helper for state changes
local function exit(self)
  if not self.current_state then
    return false, 'Uninitialized FSM'
  end
  -- Save the previous state
  self.previous_state = self.current_state
  -- Rescind the state
  self.current_state = false
  -- Without coroutines, always return true
  if not has_coroutine then return true end
  -- Stop the current coroutine
  local coro = self.current_coro
  self.current_coro = false
  -- Signal the routine to stop
  if type(coro) ~= 'thread' then return true end
  if costatus(coro) ~= 'suspended' then
    return false, 'Unknown coroutine status'
  end
  return coresume(coro, false)
end

-- enter postcondition:
-- FSM is as if it had just been initialized
-- The return of the coroutine is passed to the callback
local function enter(self, state)
  -- Check that the state exists
  if not self.transitions[state] then
    return false, 'State does not exist'
  end
  -- Stop the current state, without a callback
  -- NOTE: Callbacks are on events, not just state changes
  if self.current_state then exit(self) end
  -- Set the next state
  self.current_state = state
  -- Nothing needed if no coroutines
  if not has_coroutine then return true end
  -- Start any next state coroutine
  local loop = self.loops[state]
  if type(loop) ~= 'function' then return true end
  local coro = cocreate(loop)
  self.current_coro = coro
  return coresume(coro)
end

-- After firing an event, the FSM moves into
-- a "just created" state, per enter
local function dispatch(self, event)
  local stateA = self.current_state
  if not stateA then
    return false, 'Uninitialized FSM'
  end
  local transitions = self.transitions[stateA]
  if not transitions then
    return false, 'Bad state'
  end
  local stateB = transitions[event]
  if not stateB then
    return false, 'Invalid transition'
  end
  -- Stop the current state
  local ret = exit(self)
  -- Run the callback between the switch of states
  local callbacks = self.callbacks[stateA]
  local callback = callbacks and callbacks[event]
  if callback then
    pcall(callback, stateA, stateB, event, ret)
  end
  -- Enter the new state
  return enter(self, stateB)
end

-- After updating the current state, any current coroutine
-- is resumed, and what it yields is returned to the user
local function update(self, ...)
  if not self.current_state then
    return false, 'Uninitialized FSM'
  end
  -- Allow a coroutine to resume
  local coro = self.current_coro
  -- No coroutine is attached
  if type(coro) ~= 'thread' then return true end
  if costatus(coro) ~= 'suspended' then
    return false, 'Unknown coroutine status'
  end
  -- Signal the routine to resume
  return coresume(coro, true, ...)
end

-- Attach a coroutine to a state
local function add_coro(self, state, loop)
  if type(state) ~= 'string' then
    return false, 'Bad state name'
  end
  if type(loop) ~= 'function' then
    return false, 'Bad loop'
  end
  self.loops[state] = loop
  -- If we are in that state, then
  -- we should attach should attach ourselves
  if state == self.current_state then
    self.current_coro = cocreate(loop)
  end
end

local function add_state(self, state, loop)
  if type(state) ~= 'string' then
    return false, 'Bad state name'
  end
  -- Add the state to our transition table
  if type(self.transitions[state]) ~= 'table' then
    self.transitions[state] = {}
  end
  -- Ensure we have a table of callbacks
  if type(self.callbacks[state]) ~= 'table' then
    self.callbacks[state] = {}
  end
  -- Add the loop
  add_coro(self, state, loop)
  -- Check if we should initialize the FSM
  if type(self.current_state) ~= 'string' then
    self.current_state = state
  end
  return true
end

-- Only the callback is optional
local function add_transition(self, stateA, eventAB, stateB, callbackAB)
  if type(eventAB) ~= 'string' then
    return false, 'Bad event name'
  elseif type(stateA) ~= 'string' then
    return false, 'Bad beginning state'
  elseif type(stateB) ~= 'string' then
    return false, 'Bad destination state'
  end
  -- NOTE: Adding states must be guaranteed to work
  add_state(self, stateA)
  add_state(self, stateB)
  self.transitions[stateA][eventAB] = stateB
  -- Add a callback when state A goes to state B
  if type(callbackAB) == 'function' then
    self.callbacks[stateA][stateB] = callbackAB
  end
  return true
end

function fsm.new(transitions)
  if transitions ~= nil and type(transitions) ~= 'table' then
    return false, 'Bad input'
  end
  local obj = {
    loops = {},
    transitions = {},
    callbacks = {},
    add_state = add_state,
    add_transition = add_transition,
    -- Start/stop the machine
    enter = enter,
    update = update,
    exit = exit,
    -- Send events to the state machine
    dispatch = dispatch,
    --
    previous_state = false,
    current_state = false,
  }
  if has_coroutine then
    obj.add_coro = add_coro
    obj.current_coro = false
  end
  for _, tr in ipairs(transitions) do
    if type(tr) == 'table' then
      if #tr == 1 or #tr == 2 then
        -- Adding a state (possible coroutine)
        add_state(obj, unpack(tr))
      elseif #tr == 3 or #tr == 4 then
        -- Adding a transition (possible callback)
        add_transition(obj, unpack(tr))
      end
    end
  end
  return obj
end

return fsm